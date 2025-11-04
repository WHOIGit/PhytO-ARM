#!/usr/bin/env python3
"""
Standalone utility for manually controlling JVL motor position.

Usage:
    ./manual_motor_control.py <ip_address> <rotations> [OPTIONS]

Examples:
    # Test connection and display motor status
    ./manual_motor_control.py 192.168.13.3 0 --test

    # Move 5 rotations forward at default speed
    ./manual_motor_control.py 192.168.13.3 5.0

    # Move 2.5 rotations backward
    ./manual_motor_control.py 192.168.13.3 -2.5

    # Move with custom port and velocity
    ./manual_motor_control.py 192.168.13.3 1.0 --port 502 --velocity 50

    # Move and wait for completion
    ./manual_motor_control.py 192.168.13.3 3.0 --wait

    # Increase timeout for slow networks
    ./manual_motor_control.py 192.168.13.3 1.0 --timeout 30
"""

import argparse
import sys
import time
import threading
from pyModbusTCP.client import ModbusClient

# Import the motor register definitions
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src/jvl_motor/src'))
import mac400


COUNTS_PER_MOTOR_ROTATION = 8192  # Standard for JVL MAC400
# Note: For output shaft rotations with gearing, multiply by gear ratio


def read_registers(client, registers):
    """Read a set of registers from the motor."""
    min_addr = min(r.addr[0] for r in registers)
    max_addr = max(r.addr[-1] for r in registers)

    count = max_addr - min_addr + 1
    if count >= 125:
        raise ValueError('Register range is too wide')

    with client.lock:
        values = client.read_holding_registers(min_addr, count)

    if values is None:
        raise RuntimeError('Failed to read registers')

    output = {}
    for reg in registers:
        output[reg] = reg.decode(*[values[a - min_addr] for a in reg.addr])
    return output


def write_registers(client, reg_values):
    """Write a set of registers to the motor."""
    # Check contiguity
    for r, rnext in zip(reg_values.keys(), list(reg_values.keys())[1:]):
        if rnext.addr[0] != r.addr[-1] + 1:
            raise ValueError('Cannot write non-contiguous registers')

    encoded = []
    for r, v in reg_values.items():
        encoded.extend(r.encode(v))

    with client.lock:
        min_addr = list(reg_values.keys())[0].addr[0]
        success = client.write_multiple_registers(min_addr, encoded)

    if not success:
        raise RuntimeError('Failed to write registers')


def stop_motor(client):
    """Stop the motor by setting it to passive mode."""
    print("Stopping motor...")
    write_registers(client, {
        mac400.MODE_REG: mac400.MODE.PASSIVE
    })


def move_motor(client, rotations, velocity=20, acceleration=1200, torque=3.0, wait=False, use_velocity_mode=False, gear_ratio=1.0):
    """
    Move the motor by a specified number of rotations.

    Args:
        client: ModbusClient instance
        rotations: Number of OUTPUT shaft rotations (positive=forward, negative=backward)
        velocity: Speed in RPM (default: 20)
        acceleration: Acceleration in RPM/s (default: 1200)
        torque: Torque as percentage of rated (default: 3.0)
        wait: If True, wait until movement completes
        use_velocity_mode: If True, use velocity mode instead of position mode (mimics winch behavior)
        gear_ratio: Gear ratio between motor and output shaft (default: 1.0)
    """
    # Calculate counts per output rotation accounting for gear ratio
    counts_per_output_rotation = COUNTS_PER_MOTOR_ROTATION * gear_ratio

    # Read current position
    print("Reading current position...")
    reg_values = read_registers(client, [mac400.P_IST, mac400.MODE_REG])
    current_position = reg_values[mac400.P_IST]
    current_mode = reg_values[mac400.MODE_REG]

    current_output_rotations = current_position / counts_per_output_rotation
    print(f"Current position: {current_position} counts ({current_output_rotations:.3f} output rotations)")
    if gear_ratio != 1.0:
        print(f"  (motor: {current_position/COUNTS_PER_MOTOR_ROTATION:.2f} rotations, gear ratio: {gear_ratio}:1)")
    print(f"Current mode: {current_mode.name}")

    # Check if motor is already moving
    if current_mode != mac400.MODE.PASSIVE:
        print(f"Warning: Motor is in {current_mode.name} mode, not PASSIVE")
        response = input("Stop motor and continue? (y/n): ")
        if response.lower() != 'y':
            print("Aborted.")
            return
        stop_motor(client)
        time.sleep(0.5)

    # Calculate target position
    target_offset = int(round(rotations * counts_per_output_rotation))
    target_position = current_position + target_offset
    target_output_rotations = target_position / counts_per_output_rotation

    print(f"\nMoving {rotations:+.3f} output rotations ({target_offset:+d} counts)")
    print(f"Target position: {target_position} counts ({target_output_rotations:.3f} output rotations)")
    print(f"Velocity: {velocity} RPM")
    print(f"Acceleration: {acceleration} RPM/s")
    print(f"Torque: {torque} (rated torque)")

    # Confirm movement
    response = input("\nProceed with movement? (y/n): ")
    if response.lower() != 'y':
        print("Aborted.")
        return

    if use_velocity_mode:
        # Use velocity mode - mimics winch behavior
        print("\nUsing VELOCITY mode (like winch)...")
        direction_rpm = velocity if rotations > 0 else -velocity
        print(f"Register values being written:")
        print(f"  MODE_REG: VELOCITY (1)")
        print(f"  V_SOLL: {direction_rpm:+.1f} RPM (encoded: {int(direction_rpm * 2.77056)})")
        print(f"  A_SOLL: {acceleration} RPM/s (encoded: {int(acceleration * 3.598133e-3)})")
        print(f"  T_SOLL: {torque} (encoded: {int(torque * 341)})")

        write_registers(client, {
            mac400.MODE_REG:  mac400.MODE.VELOCITY,
            mac400.P_SOLL:    0,
            mac400.P_NEW:     0,
            mac400.V_SOLL:    direction_rpm,
            mac400.A_SOLL:    acceleration,
            mac400.T_SOLL:    torque,
        })

        print("\nCommand sent successfully!")
        print("Motor will move at constant velocity until stopped.")
        print("Monitoring position to auto-stop at target...")

        # Monitor and stop when target reached
        try:
            while True:
                time.sleep(0.1)
                reg_values = read_registers(client, [mac400.P_IST, mac400.V_IST])
                position = reg_values[mac400.P_IST]
                actual_velocity = reg_values[mac400.V_IST]

                error = target_position - position
                output_rot = position / counts_per_output_rotation
                print(f"Position: {position} counts ({output_rot:.3f} out rot), "
                      f"Error: {error:+d} counts, Velocity: {actual_velocity:+.1f} RPM", end='\r')

                # Stop when within 50 counts of target
                if abs(error) < 50:
                    print(f"\n\nTarget reached! Stopping motor...")
                    stop_motor(client)
                    time.sleep(0.2)
                    # Read final position
                    reg_values = read_registers(client, [mac400.P_IST])
                    final_position = reg_values[mac400.P_IST]
                    final_error = target_position - final_position
                    final_output_rot = final_position / counts_per_output_rotation
                    print(f"Final position: {final_position} counts ({final_output_rot:.3f} output rotations)")
                    print(f"Final error: {final_error:+d} counts ({final_error/counts_per_output_rotation:+.4f} output rotations)")
                    break

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            stop_motor(client)

    else:
        # Use position mode - traditional approach
        print("\nUsing POSITION mode...")
        print(f"Register values being written:")
        print(f"  MODE_REG: POSITION (2)")
        print(f"  P_SOLL: {target_position}")
        print(f"  P_NEW: 0")
        print(f"  V_SOLL: {velocity} RPM (encoded: {int(velocity * 2.77056)})")
        print(f"  A_SOLL: {acceleration} RPM/s (encoded: {int(acceleration * 3.598133e-3)})")
        print(f"  T_SOLL: {torque} (encoded: {int(torque * 341)})")

        write_registers(client, {
            mac400.MODE_REG:  mac400.MODE.POSITION,
            mac400.P_SOLL:    target_position,
            mac400.P_NEW:     0,
            mac400.V_SOLL:    velocity,
            mac400.A_SOLL:    acceleration,
            mac400.T_SOLL:    torque,
        })

        print("\nCommand sent successfully!")

    if wait:
        print("\nWaiting for movement to complete...")
        print("(Press Ctrl+C to stop waiting)\n")

        try:
            while True:
                time.sleep(0.5)
                reg_values = read_registers(client, [mac400.P_IST, mac400.MODE_REG, mac400.V_IST])
                position = reg_values[mac400.P_IST]
                mode = reg_values[mac400.MODE_REG]
                actual_velocity = reg_values[mac400.V_IST]

                error = target_position - position
                output_rot = position / counts_per_output_rotation
                print(f"Position: {position} counts ({output_rot:.3f} out rot), "
                      f"Error: {error:+d} counts, Velocity: {actual_velocity:.1f} RPM, Mode: {mode.name}", end='\r')

                # Check if we've reached the target (within tolerance)
                if abs(error) < 50 and mode == mac400.MODE.PASSIVE:
                    print(f"\n\nMovement complete!")
                    print(f"Final position: {position} counts ({output_rot:.3f} output rotations)")
                    break

                # Check if motor stopped unexpectedly
                if mode == mac400.MODE.PASSIVE and abs(error) >= 50:
                    print(f"\n\nWarning: Motor stopped before reaching target!")
                    print(f"Final position: {position} counts ({output_rot:.3f} output rotations)")
                    print(f"Error: {error} counts ({error/counts_per_output_rotation:.3f} output rotations)")
                    break

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            stop_motor(client)


def verify_connection(client, retries=5, retry_delay=1.0):
    """
    Verify connection by attempting to read a register.

    Args:
        client: ModbusClient instance
        retries: Number of connection attempts (default: 5)
        retry_delay: Seconds to wait between retries (default: 1.0)

    Returns:
        dict: Register values if successful

    Raises:
        RuntimeError: If all connection attempts fail
    """
    for attempt in range(1, retries + 1):
        try:
            print(f"Attempting to read motor status (attempt {attempt}/{retries})...", end='')
            reg_values = read_registers(client, [mac400.P_IST, mac400.MODE_REG])
            print(" success!")
            return reg_values
        except Exception as e:
            print(f" failed ({e})")
            if attempt < retries:
                print(f"Retrying in {retry_delay}s...")
                time.sleep(retry_delay)
            else:
                raise RuntimeError(f"Failed to connect after {retries} attempts")


def test_motor_connection(client, gear_ratio=1.0):
    """Test motor connection and display status information."""
    print("\n=== Motor Connection Test ===\n")

    # Verify we can read registers
    reg_values = verify_connection(client)

    # Display motor information
    position = reg_values[mac400.P_IST]
    mode = reg_values[mac400.MODE_REG]

    counts_per_output_rotation = COUNTS_PER_MOTOR_ROTATION * gear_ratio
    output_rotations = position / counts_per_output_rotation

    print(f"✓ Connection successful!")
    print(f"\nMotor Status:")
    print(f"  Current Position: {position} counts ({output_rotations:.3f} output rotations)")
    if gear_ratio != 1.0:
        print(f"    (motor: {position/COUNTS_PER_MOTOR_ROTATION:.2f} rotations, gear ratio: {gear_ratio}:1)")
    print(f"  Current Mode: {mode.name} ({mode.value})")

    # Try to read additional registers for more info
    try:
        extra_regs = read_registers(client, [mac400.V_IST, mac400.U_24V,
                                              mac400.MIN_P_IST, mac400.MAX_P_IST])
        velocity = extra_regs[mac400.V_IST]
        voltage = extra_regs[mac400.U_24V]
        min_pos = extra_regs[mac400.MIN_P_IST]
        max_pos = extra_regs[mac400.MAX_P_IST]

        print(f"  Current Velocity: {velocity:.2f} RPM")
        print(f"  24V Supply: {voltage:.2f} V")
        print(f"  Position Envelope: MIN={min_pos} ({min_pos/COUNTS_PER_ROTATION:.2f} rot), "
              f"MAX={max_pos} ({max_pos/COUNTS_PER_ROTATION:.2f} rot)")

        if min_pos != 0 or max_pos != 0:
            print(f"  ⚠️  Warning: Position envelope is set! This may constrain movement.")
    except Exception as e:
        print(f"  (Could not read additional registers: {e})")

    print("\n✓ Motor is responding normally")


def main():
    parser = argparse.ArgumentParser(
        description='Manual control utility for JVL motor position',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('ip', help='IP address of the motor')
    parser.add_argument('rotations', type=float,
                       help='Number of rotations to move (positive or negative)')
    parser.add_argument('--port', type=int, default=502,
                       help='Modbus TCP port (default: 502)')
    parser.add_argument('--velocity', type=float, default=20,
                       help='Velocity in RPM (default: 20)')
    parser.add_argument('--acceleration', type=float, default=1200,
                       help='Acceleration in RPM/s (default: 1200)')
    parser.add_argument('--torque', type=float, default=3.0,
                       help='Torque as percentage of rated (default: 3.0)')
    parser.add_argument('--wait', action='store_true',
                       help='Wait for movement to complete')
    parser.add_argument('--stop', action='store_true',
                       help='Just stop the motor (ignore rotations)')
    parser.add_argument('--test', action='store_true',
                       help='Test connection and display motor status (no movement)')
    parser.add_argument('--timeout', type=float, default=10.0,
                       help='Connection timeout in seconds (default: 10.0)')
    parser.add_argument('--clear-envelope', action='store_true',
                       help='Clear position envelope limits before moving')
    parser.add_argument('--use-velocity-mode', action='store_true',
                       help='Use velocity mode instead of position mode (mimics winch behavior)')
    parser.add_argument('--gear-ratio', type=float, default=60.0,
                       help='Gear ratio between motor and output shaft (default: 60.0)')

    args = parser.parse_args()

    # Connect to motor
    print(f"Connecting to motor at {args.ip}:{args.port}...")
    client = ModbusClient(
        host=args.ip,
        port=args.port,
        auto_open=True,
        timeout=args.timeout
    )
    client.lock = threading.Lock()

    # Verify connection by attempting to read registers
    try:
        verify_connection(client)
        print("Connection verified!\n")
    except RuntimeError as e:
        print(f"\nError: {e}")
        print(f"Could not connect to motor at {args.ip}:{args.port}")
        print("\nTroubleshooting:")
        print("  - Verify motor is powered on")
        print("  - Check IP address and network connectivity")
        print("  - Ensure motor is configured for Modbus TCP")
        print(f"  - Try increasing timeout with --timeout (current: {args.timeout}s)")
        sys.exit(1)

    # Clear position envelope if requested
    if args.clear_envelope and not args.test:
        print("Clearing position envelope...")
        try:
            write_registers(client, {mac400.MIN_P_IST: 0})
            write_registers(client, {mac400.MAX_P_IST: 0})
            print("✓ Position envelope cleared\n")
        except Exception as e:
            print(f"Warning: Could not clear envelope: {e}\n")

    try:
        if args.test:
            test_motor_connection(client, gear_ratio=args.gear_ratio)
        elif args.stop:
            stop_motor(client)
            print("Motor stopped.")
        else:
            move_motor(
                client,
                args.rotations,
                velocity=args.velocity,
                acceleration=args.acceleration,
                torque=args.torque,
                wait=args.wait,
                use_velocity_mode=args.use_velocity_mode,
                gear_ratio=args.gear_ratio
            )
    except KeyboardInterrupt:
        print("\n\nInterrupted by user, stopping motor...")
        stop_motor(client)
    except Exception as e:
        print(f"\nError: {e}")
        print("Attempting to stop motor...")
        try:
            stop_motor(client)
        except:
            pass
        sys.exit(1)
    finally:
        client.close()


if __name__ == '__main__':
    main()
