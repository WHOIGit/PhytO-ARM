import os
import sys
import unittest

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import mac400 as reg
from mac400_model import MotorModel


# Clock inputs are integer nanoseconds
SECOND = 10**9


class TestMotorModel(unittest.TestCase):
    def motor(self, mode=reg.MODE.VELOCITY, speed=10, accel=10, target=0):
        # At 60 counts per turn, one RPM is one encoder count per second
        model = MotorModel(counts_per_turn=60)
        model.write({
            reg.A_SOLL: accel,
            reg.MODE_REG: mode,
            reg.P_SOLL: target,
            reg.V_SOLL: speed,
        })
        return model

    def test_writing_the_encoder_preserves_shaft_travel(self):
        model = self.motor(speed=10, accel=0)

        # After 1.05 seconds the shaft has traveled a fractional encoder count
        start = SECOND
        model.advance(start)
        model.advance(start + 1_050_000_000)
        self.assertEqual(model.shaft_position, 10.5)

        for reference in (0, -20):
            model.write({reg.P_IST: reference})
            self.assertEqual(model.read(reg.P_IST), reference)
            self.assertEqual(model.shaft_position, 10.5)

        model.advance(start + 2_050_000_000)
        self.assertEqual(model.shaft_position, 20.5)
        self.assertEqual(model.read(reg.P_IST), -10)

    def test_first_clock_pause_and_rewind(self):
        model = self.motor()
        start = 10**18
        end = start + 2 * SECOND

        # The initial ROS timestamp can be much larger than the first interval
        model.advance(0)
        model.advance(start)
        self.assertEqual(model.read(reg.P_IST), 0)

        # One second of acceleration and one at full speed give 5 + 10 counts
        model.advance(end)
        self.assertEqual(model.read(reg.P_IST), 15)
        model.advance(end)
        self.assertEqual(model.read(reg.P_IST), 15)

        # The previous velocity command ends when the clock rewinds
        model.advance(SECOND)
        self.assertEqual(model.read(reg.MODE_REG), reg.MODE.PASSIVE)
        model.advance(end)
        self.assertEqual(model.read(reg.P_IST), 15)

    def test_rewind_to_zero_requires_a_new_starting_time(self):
        model = self.motor(speed=10, accel=0)
        model.advance(SECOND)
        model.advance(2 * SECOND)

        # A rewind to zero stops the shaft where it was
        model.advance(0)
        self.assertEqual(model.read(reg.MODE_REG), reg.MODE.PASSIVE)
        self.assertEqual(model.read(reg.P_IST), 10)

        # The first nonzero tick sets the baseline for the new command
        model.write({reg.MODE_REG: reg.MODE.VELOCITY})
        model.advance(20 * SECOND)
        self.assertEqual(model.read(reg.P_IST), 10)
        model.advance(21 * SECOND)
        self.assertEqual(model.read(reg.P_IST), 20)

    def test_enabling_motor_after_idle_uses_last_tick(self):
        model = self.motor(mode=reg.MODE.PASSIVE, speed=10, accel=0)
        model.advance(SECOND)
        model.advance(60 * SECOND)

        # Only the second since the last passive tick belongs to this move
        model.write({reg.MODE_REG: reg.MODE.VELOCITY})
        model.advance(61 * SECOND)
        self.assertEqual(model.read(reg.P_IST), 10)

        # A new speed setpoint takes effect only when the clock advances
        model.write({reg.V_SOLL: 20})
        model.advance(61 * SECOND)
        self.assertEqual(model.read(reg.V_IST_16), 10)
        model.advance(62 * SECOND)
        self.assertEqual(model.read(reg.V_IST_16), 20)

    def test_zero_torque_from_rest_and_reenable(self):
        for mode in (reg.MODE.VELOCITY, reg.MODE.POSITION):
            model = self.motor(mode, speed=10, accel=10, target=100)
            model.write({reg.T_SOLL: 0})
            model.advance(SECOND)
            model.advance(6 * SECOND)
            self.assertEqual(model.read(reg.P_IST), 0)
            self.assertEqual(model.read(reg.V_IST_16), 0)
            self.assertEqual(model.read(reg.MODE_REG), mode)

            # Restoring torque lets the same command accelerate the shaft
            model.write({reg.T_SOLL: 1})
            model.advance(7 * SECOND)
            self.assertEqual(model.read(reg.P_IST), 5)
            self.assertEqual(model.read(reg.V_IST_16), 10)

    def test_zero_torque_coasts_through_target_and_respects_limit(self):
        for sign in (-1, 1):
            for ticks in (1, 100):
                model = self.motor(speed=sign * 10, accel=10)

                # Reach ±10 counts per second at position ±5 before coasting
                model.advance(SECOND)
                model.advance(2 * SECOND)

                bound = reg.MAX_P_IST if sign > 0 else reg.MIN_P_IST
                model.write({
                    bound: sign * 20,
                    reg.MODE_REG: reg.MODE.POSITION,
                    reg.P_SOLL: sign * 8,
                    reg.T_SOLL: 0,
                })
                for step in range(1, ticks + 1):
                    model.advance(2 * SECOND + step * SECOND // ticks)
                self.assertEqual(model.read(reg.P_IST), sign * 15)
                self.assertEqual(model.read(reg.V_IST_16), sign * 10)
                self.assertEqual(model.read(reg.MODE_REG), reg.MODE.POSITION)
                model.advance(4 * SECOND)
                self.assertEqual(model.read(reg.P_IST), sign * 21)
                self.assertEqual(model.read(reg.MODE_REG), reg.MODE.PASSIVE)

    def test_envelope_on_reversal_between_observations(self):
        for sign in (-1, 1):
            model = self.motor(speed=sign * 10)
            model.advance(SECOND)
            model.advance(2 * SECOND)

            # Braking from speed 10 at count 5 crosses the limit at count 8
            bound = reg.MAX_P_IST if sign > 0 else reg.MIN_P_IST
            model.write({bound: sign * 8, reg.V_SOLL: -sign * 10})
            model.advance(32 * SECOND)
            self.assertEqual(model.read(reg.P_IST), sign * 9)
            errors = reg.implode_bits(model.read(reg.ERR_STAT))
            self.assertEqual(errors, (1 << 7) | (1 << 24))
            self.assertEqual(model.read(reg.MODE_REG), reg.MODE.PASSIVE)

    def test_exact_limit_is_legal_including_at_position_target(self):
        for sign, bound in ((1, reg.MAX_P_IST), (-1, reg.MIN_P_IST)):
            model = self.motor(
                reg.MODE.POSITION, speed=10, accel=0, target=sign * 10
            )
            model.write({bound: sign * 10})
            model.advance(SECOND)
            at_limit = 2 * SECOND
            model.advance(at_limit)
            self.assertEqual(model.read(reg.P_IST), sign * 10)
            self.assertEqual(model.read(reg.MODE_REG), reg.MODE.POSITION)

            # The encoder advances one count after crossing the half-count mark
            model.write({reg.MODE_REG: reg.MODE.VELOCITY, reg.V_SOLL: sign})
            model.advance(at_limit + 400_000_000)
            self.assertEqual(model.read(reg.P_IST), sign * 10)
            self.assertEqual(model.read(reg.MODE_REG), reg.MODE.VELOCITY)
            model.advance(at_limit + 600_000_000)
            self.assertEqual(model.read(reg.P_IST), sign * 11)
            self.assertEqual(model.read(reg.MODE_REG), reg.MODE.PASSIVE)

    def test_stationary_trip_and_latched_fault(self):
        for sign, bound in ((1, reg.MAX_P_IST), (-1, reg.MIN_P_IST)):
            encoder = {reg.P_IST: sign * 25}
            limit = {bound: sign * 20}
            for writes in ((limit, encoder), (encoder, limit)):
                model = MotorModel()
                for values in writes:
                    model.write(values)

                # A trip retains the current encoder count
                self.assertEqual(model.read(reg.P_IST), sign * 25)
                self.assertTrue(model.read(reg.ERR_STAT)[7])

                # Neither removing the limit nor commanding inward clears it
                model.write({
                    bound: 0,
                    reg.MODE_REG: reg.MODE.VELOCITY,
                    reg.V_SOLL: -sign,
                })
                model.advance(SECOND)
                model.advance(11 * SECOND)
                self.assertEqual(model.read(reg.P_IST), sign * 25)
                self.assertEqual(model.read(reg.MODE_REG), reg.MODE.PASSIVE)
                self.assertTrue(model.read(reg.ERR_STAT)[7])
                self.assertFalse(MotorModel().read(reg.ERR_STAT)[7])

    def test_enabling_limit_after_signed_encoder_rollover(self):
        for sign, bound in ((1, reg.MAX_P_IST), (-1, reg.MIN_P_IST)):
            model = self.motor(speed=sign, accel=0)
            edge = 2**31 - 1 if sign > 0 else -(2**31)
            model.write({reg.P_IST: edge})

            # The signed register wraps after a single count of shaft travel
            model.advance(SECOND)
            model.advance(2 * SECOND)
            wrapped = -(2**31) if sign > 0 else 2**31 - 1
            self.assertEqual(model.read(reg.P_IST), wrapped)
            self.assertEqual(model.shaft_position, sign)

            # A newly enabled limit sees the continuous position and trips here
            model.write({bound: sign * 100})
            self.assertTrue(model.read(reg.ERR_STAT)[7])
            self.assertEqual(model.read(reg.MODE_REG), reg.MODE.PASSIVE)
            model.advance(3 * SECOND)
            self.assertEqual(model.read(reg.P_IST), wrapped)
            self.assertEqual(model.shaft_position, sign)

    def test_retarget_inside_stopping_distance_holds_before_envelope(self):
        for sign in (-1, 1):
            for partitions in (1, 100):
                model = self.motor(speed=sign * 20, accel=10)
                model.advance(SECOND)

                # Reach count ±20 with another 20 counts of braking distance
                model.advance(3 * SECOND)
                bound = reg.MAX_P_IST if sign > 0 else reg.MIN_P_IST
                model.write({
                    bound: sign * 22,
                    reg.MODE_REG: reg.MODE.POSITION,
                    reg.P_SOLL: sign * 21,
                })
                for step in range(1, partitions + 1):
                    elapsed = step * 5 * SECOND // partitions
                    model.advance(3 * SECOND + elapsed)
                self.assertEqual(model.read(reg.P_IST), sign * 21)
                self.assertEqual(model.read(reg.V_IST_16), 0)
                self.assertEqual(model.read(reg.MODE_REG), reg.MODE.POSITION)
                self.assertEqual(reg.implode_bits(model.read(reg.ERR_STAT)), 0)

    def test_position_move_accelerates_cruises_and_brakes(self):
        model = self.motor(reg.MODE.POSITION, speed=10, accel=10, target=30)
        model.advance(SECOND)

        # Ramping to speed 10 takes one second and covers five counts
        model.advance(2 * SECOND)
        self.assertEqual(model.shaft_position, 5)
        self.assertEqual(model.read(reg.V_IST_16), 10)

        # Two seconds at full speed leave five counts for braking
        model.advance(4 * SECOND)
        self.assertEqual(model.shaft_position, 25)
        self.assertEqual(model.read(reg.V_IST_16), 10)

        # Halfway through braking, speed is 5 and 1.25 counts remain
        model.advance(4 * SECOND + SECOND // 2)
        self.assertEqual(model.shaft_position, 28.75)
        self.assertEqual(model.read(reg.V_IST_16), 5)
        model.advance(5 * SECOND)
        self.assertEqual(model.shaft_position, 30)
        self.assertEqual(model.read(reg.V_IST_16), 0)

    def test_position_mode_stops_at_target_in_either_direction(self):
        # One large clock step and 100 small steps should reach the same state
        for target in (-100, -3, 3, 100):
            coarse = self.motor(reg.MODE.POSITION, target=target)
            fine = self.motor(reg.MODE.POSITION, target=target)
            coarse.advance(SECOND)
            fine.advance(SECOND)
            for block in range(12):
                stamp = (1 + block) * SECOND
                for step in range(1, 101):
                    fine.advance(stamp + step * SECOND // 100)
                coarse.advance(stamp + SECOND)
                self.assertAlmostEqual(coarse.shaft_position,
                                       fine.shaft_position, places=5)
                self.assertAlmostEqual(
                    coarse.read(reg.V_IST_16), fine.read(reg.V_IST_16), places=5
                )
            self.assertEqual(coarse.read(reg.P_IST), target)
            self.assertEqual(coarse.read(reg.V_IST_16), 0)
            self.assertEqual(coarse.read(reg.MODE_REG), reg.MODE.POSITION)
