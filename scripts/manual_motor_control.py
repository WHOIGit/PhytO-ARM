#!/usr/bin/env python3
"""
Standalone utility for manually controlling JVL motor position.

Usage:
    ./manual_motor_control.py <ip_address> <rotations> [--port PORT] [--velocity RPM] [--wait]

Examples:
    # Move 5 rotations forward at default speed
    ./manual_motor_control.py 192.168.13.3 5.0

    # Move 2.5 rotations backward
    ./manual_motor_control.py 192.168.13.3 -2.5

    # Move with custom port and velocity
    ./manual_motor_control.py 192.168.13.3 1.0 --port 502 --velocity 50

    # Move and wait for completion
    ./manual_motor_control.py 192.168.13.3 3.0 --wait
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


COUNTS_PER_ROTATION = 8192  # Standard for JVL MAC400


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


def move_motor(client, rotations, velocity=20, acceleration=1200, torque=3.0, wait=False):
    """
    Move the motor by a specified number of rotations.

    Args:
        client: ModbusClient instance
        rotations: Number of rotations (positive=forward, negative=backward)
        velocity: Speed in RPM (default: 20)
        acceleration: Acceleration in RPM/s (default: 1200)
        torque: Torque as percentage of rated (default: 3.0)
        wait: If True, wait until movement completes
    """
    # Read current position
    print("Reading current position...")
    reg_values = read_registers(client, [mac400.P_IST, mac400.MODE_REG])
    current_position = reg_values[mac400.P_IST]
    current_mode = reg_values[mac400.MODE_REG]

    print(f"Current position: {current_position} counts ({current_position/COUNTS_PER_ROTATION:.2f} rotations)")
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
    target_offset = int(round(rotations * COUNTS_PER_ROTATION))
    target_position = current_position + target_offset

    print(f"\nMoving {rotations:+.2f} rotations ({target_offset:+d} counts)")
    print(f"Target position: {target_position} counts ({target_position/COUNTS_PER_ROTATION:.2f} rotations)")
    print(f"Velocity: {velocity} RPM")
    print(f"Acceleration: {acceleration} RPM/s")
    print(f"Torque: {torque} (rated torque)")

    # Confirm movement
    response = input("\nProceed with movement? (y/n): ")
    if response.lower() != 'y':
        print("Aborted.")
        return

    # Send position command
    print("\nSending position command...")
    write_registers(client, {
        mac400.MODE_REG:  mac400.MODE.POSITION,
        mac400.P_SOLL:    target_position,
        mac400.P_NEW:     0,
        mac400.V_SOLL:    velocity,
        mac400.A_SOLL:    acceleration,
        mac400.T_SOLL:    torque,
    })

    print("Command sent successfully!")

    if wait:
        print("\nWaiting for movement to complete...")
        print("(Press Ctrl+C to stop waiting)\n")

        try:
            while True:
                time.sleep(0.5)
                reg_values = read_registers(client, [mac400.P_IST, mac400.MODE_REG])
                position = reg_values[mac400.P_IST]
                mode = reg_values[mac400.MODE_REG]

                error = target_position - position
                print(f"Position: {position} counts ({position/COUNTS_PER_ROTATION:.2f} rot), "
                      f"Error: {error:+d} counts, Mode: {mode.name}", end='\r')

                # Check if we've reached the target (within tolerance)
                if abs(error) < 50 and mode == mac400.MODE.PASSIVE:
                    print(f"\n\nMovement complete!")
                    print(f"Final position: {position} counts ({position/COUNTS_PER_ROTATION:.2f} rotations)")
                    break

                # Check if motor stopped unexpectedly
                if mode == mac400.MODE.PASSIVE and abs(error) >= 50:
                    print(f"\n\nWarning: Motor stopped before reaching target!")
                    print(f"Final position: {position} counts ({position/COUNTS_PER_ROTATION:.2f} rotations)")
                    print(f"Error: {error} counts ({error/COUNTS_PER_ROTATION:.3f} rotations)")
                    break

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            stop_motor(client)


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

    args = parser.parse_args()

    # Connect to motor
    print(f"Connecting to motor at {args.ip}:{args.port}...")
    client = ModbusClient(
        host=args.ip,
        port=args.port,
        auto_open=True,
        timeout=5.0
    )
    client.lock = threading.Lock()

    if not client.is_open:
        print(f"Error: Could not connect to motor at {args.ip}:{args.port}")
        sys.exit(1)

    print("Connected successfully!\n")

    try:
        if args.stop:
            stop_motor(client)
            print("Motor stopped.")
        else:
            move_motor(
                client,
                args.rotations,
                velocity=args.velocity,
                acceleration=args.acceleration,
                torque=args.torque,
                wait=args.wait
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
