# MAC400 simulator

For a deployment with a winch and instruments, see the [PhytO-ARM simulation guide](../../SIMULATION.md).

## Run the driver

In a sourced ROS Noetic workspace, launch the production JVL driver and the
simulated motor together.

```bash
roslaunch jvl_motor sim_motor.launch
```

The motor starts in passive mode. In a second terminal with the same workspace
sourced, command 60 RPM with 1200 RPM/s acceleration and rated torque.

```bash
rosservice call /motor/set_velocity 60 1200 1
rostopic echo -n 3 /motor/motion
rosservice call /motor/stop
```

`position` is in encoder counts (8192 per revolution) and `velocity` is in RPM.
The driver polls at 1 Hz over Modbus/TCP at `127.0.0.1:1502`. Set
`refresh_rate:=<hz>` or `motor_modbus_port:=<number>` on the launch command to change them.

## Motor behavior

By default, motion follows wall time. To use Gazebo or another ROS clock, set
`/use_sim_time` before launch and supply `/clock`. Pausing the clock holds the
encoder position while the driver can still send commands. Rewinding the clock
stops the motor at its last position.

The model assumes an unloaded shaft and reports nominal supply voltages. With
zero torque a moving shaft coasts at its current speed. Any positive torque
limit permits the commanded acceleration.

The simulator uses continuous encoder counts for shaft travel and position
limit checks. Reading `P_IST` gives a signed 32-bit count that wraps on
overflow. Zeroing the encoder changes its reference while the shaft stays put.

The shaft may reach either enabled limit. Crossing it latches a fault and puts
the drive in passive mode. Setting a limit that already excludes the current
position stops the shaft where it is. A zero limit disables that bound.
Relaunch the pair to clear the fault and simulate a power cycle.

## Test the driver

From a sourced development workspace, run the real driver against the simulator
with a stepped ROS clock.

```bash
rostest jvl_motor mac400_sim.test
```
