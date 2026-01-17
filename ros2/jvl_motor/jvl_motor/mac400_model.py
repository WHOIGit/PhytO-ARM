import math

from jvl_motor import mac400


# Allow for floating-point roundoff at the braking point and target, in counts
POSITION_TOLERANCE = 1e-6


# Model the unloaded shaft in encoder counts and seconds
class MotorModel:
    def __init__(self, counts_per_turn=8192):
        self.counts_per_rpm_second = counts_per_turn / 60.0

        # Keep continuous encoder counts and velocity in counts per second
        self._encoder_zero = 0.0
        self._limit_tripped = False
        self._position = 0.0
        self._stamp_ns = 0
        self._velocity = 0.0

        # Store the commanded registers in the driver's engineering units.
        # The initial T_SOLL value encodes to 1023 on the wire.
        self._registers = {
            mac400.A_SOLL: 0.0,
            mac400.MAX_P_IST: 0,
            mac400.MIN_P_IST: 0,
            mac400.MODE_REG: mac400.MODE.PASSIVE,
            mac400.P_NEW: 0,
            mac400.P_SOLL: 0,
            mac400.T_SOLL: 3.0,
            mac400.V_SOLL: 0.0,
        }

    # Return physical shaft travel across changes to the encoder reference
    @property
    def shaft_position(self):
        return self._encoder_zero + self._position

    # Return the register value in the engineering units used by mac400
    def read(self, register):
        # Only the P_IST readback wraps at the signed 32-bit boundary
        if register == mac400.P_IST:
            return (round(self._position) + 2**31) % 2**32 - 2**31

        # Convert the simulated shaft velocity to RPM for the register codec
        if register == mac400.V_IST_16:
            return self._velocity / self.counts_per_rpm_second

        # A position limit trip sets PLIM_ERR and ANY_ERR in register 35
        if register == mac400.ERR_STAT:
            flags = (1 << 7) | (1 << 24) if self._limit_tripped else 0
            return mac400.explode_bits(flags, 32)

        return self._registers[register]

    # Apply all setpoints in a single Modbus write before checking the limits
    def write(self, values):
        # Writing the encoder shifts its reference without moving the shaft
        registers = values.copy()
        if mac400.P_IST in registers:
            position = registers.pop(mac400.P_IST)
            self._encoder_zero = self.shaft_position - position
            self._position = position
        self._registers.update(registers)

        # A write can put the current position outside the new limits. Trip
        # in place, using the same continuous encoder counts as for motion.
        if self._first_illegal_count(round(self._position)) is not None:
            self._limit_tripped = True

        # STOP and PASSIVE halt the shaft immediately. A latched limit fault
        # also holds the drive passive on subsequent commands.
        mode = self._registers[mac400.MODE_REG]
        stop_requested = mode in (mac400.MODE.PASSIVE, mac400.MODE.STOP)
        if stop_requested or self._limit_tripped:
            self._stop()

    # Each limit is inclusive and a zero register disables that limit
    def _first_illegal_count(self, count):
        low = self._registers[mac400.MIN_P_IST]
        high = self._registers[mac400.MAX_P_IST]
        if low and count < low:
            return low - 1
        if high and count > high:
            return high + 1
        return None

    def _stop(self):
        self._velocity = 0.0
        self._registers[mac400.MODE_REG] = mac400.MODE.PASSIVE

    # Advance the motor to the given ROS timestamp in nanoseconds
    def advance(self, stamp_ns):
        # Record every tick so the next move excludes time spent idle
        previous_ns, self._stamp_ns = self._stamp_ns, stamp_ns

        # The first nonzero ROS tick establishes the starting time.
        # A repeated tick contributes no elapsed time.
        if previous_ns == 0 or stamp_ns == previous_ns:
            return

        # A backward clock step stops the drive at its current position
        dt = (stamp_ns - previous_ns) / 1e9
        if dt < 0:
            self._stop()
            return

        # A passive drive holds its position as time advances
        mode = self._registers[mac400.MODE_REG]
        if mode == mac400.MODE.PASSIVE:
            return

        # With no load or friction, the unpowered shaft coasts
        if self._registers[mac400.T_SOLL] == 0:
            self._segment(dt, self._velocity)
            return

        # Convert RPM and RPM/s to encoder counts/s and counts/s²
        scale = self.counts_per_rpm_second
        speed = self._registers[mac400.V_SOLL] * scale
        acceleration = abs(self._registers[mac400.A_SOLL]) * scale

        # Position mode stops at its target and velocity mode ramps to set speed
        if mode == mac400.MODE.POSITION:
            self._position_move(dt, abs(speed), acceleration)
        else:
            remaining = self._ramp(dt, speed, acceleration)
            self._segment(remaining, self._velocity)

    # Advance one interval of constant acceleration in one direction
    def _segment(self, dt, end_velocity):
        # A tick may end exactly at the boundary between two phases
        if dt <= 0:
            return

        # A limit trip in an earlier phase cancels the rest of the move
        mode = self._registers[mac400.MODE_REG]
        if mode == mac400.MODE.PASSIVE:
            return

        # The mean velocity gives the distance under constant acceleration
        delta = (self._velocity + end_velocity) * 0.5 * dt
        new_position = self._position + delta

        # A powered position move holds at the target if this step crosses it
        if mode == mac400.MODE.POSITION and self._registers[mac400.T_SOLL] > 0:
            target = self._registers[mac400.P_SOLL]
            low, high = sorted((self._position, new_position))
            if low <= target <= high:
                new_position, end_velocity = float(target), 0.0

        # A limit fault stops the shaft at the first count outside the envelope
        trip_count = self._first_illegal_count(round(new_position))
        if trip_count is not None:
            self._position = float(trip_count)
            self._limit_tripped = True
            self._stop()
            return

        self._position, self._velocity = new_position, end_velocity

    # Accelerate to speed, returning the time left after reaching it
    def _ramp(self, dt, target_velocity, acceleration):
        # A zero acceleration setting applies the new velocity immediately
        if not acceleration:
            self._velocity = target_velocity
            return dt

        # On reversal, check the turning point for a limit crossing before
        # starting the ramp in the opposite direction.
        if self._velocity * target_velocity < 0:
            dt = self._ramp(dt, 0.0, acceleration)
            if dt == 0:
                return 0.0

        # A partial ramp uses the entire tick
        change = target_velocity - self._velocity
        ramp_time = abs(change) / acceleration
        if dt < ramp_time:
            velocity_change = math.copysign(acceleration * dt, change)
            self._segment(dt, self._velocity + velocity_change)
            return 0.0

        # A completed ramp leaves time for the next phase unless a limit trips
        self._segment(ramp_time, target_velocity)
        if self._registers[mac400.MODE_REG] == mac400.MODE.PASSIVE:
            return 0.0
        return dt - ramp_time

    # Advance through as many phases of the position move as fit in this tick
    def _position_move(self, dt, max_speed, acceleration):
        # Hold position when the encoder has already reached the target
        target = self._registers[mac400.P_SOLL]
        displacement = target - self._position
        if not displacement:
            self._velocity = 0.0
            return

        # With no acceleration ramp the motor can start at full speed
        direction = math.copysign(1, displacement)
        if not acceleration:
            self._velocity = direction * max_speed
            self._segment(dt, self._velocity)
            return

        # Stop any travel away from the new target before planning the move
        if direction * self._velocity < 0:
            dt = self._ramp(dt, 0.0, acceleration)
            if dt == 0:
                return

        # A target inside the stopping distance is held as soon as we cross it
        distance = abs(target - self._position)
        braking_distance = self._velocity**2 / (2 * acceleration)
        if braking_distance > distance + POSITION_TOLERANCE:
            self._ramp(dt, 0.0, acceleration)
            return

        # Solve distance = (peak² - v²)/(2a) + peak²/(2a) for a ramp up
        # followed by braking to rest. The commanded speed caps the peak.
        peak_speed = math.sqrt(acceleration * distance + self._velocity**2 / 2)
        peak_speed = min(max_speed, peak_speed)
        remaining = self._ramp(dt, direction * peak_speed, acceleration)
        if not remaining or not peak_speed:
            return

        # Cruise until the braking point, then decelerate to the target
        braking_distance = peak_speed**2 / (2 * acceleration)
        distance = abs(target - self._position)
        cruise_distance = max(0, distance - braking_distance)
        cruise_time = min(remaining, cruise_distance / peak_speed)
        self._segment(cruise_time, self._velocity)
        self._ramp(remaining - cruise_time, 0.0, acceleration)

        # Remove sub-count roundoff only if the move ended without a fault
        if (self._registers[mac400.MODE_REG] == mac400.MODE.POSITION and
                direction * (target - self._position) <= POSITION_TOLERANCE):
            self._position, self._velocity = float(target), 0.0
