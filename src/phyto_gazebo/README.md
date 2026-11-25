# Gazebo winch simulator

The world contains a fixed barge with a winch and a suspended IFCB and CTD.
Gazebo applies tension at the lifting eye when the line is stretched. Paying
out extra line lets the payload fall until the line catches it again.

## Move to a depth

From a sourced ROS Noetic workspace with Gazebo Classic 11, launch the world,
the simulated MAC400 and the production winch and motor nodes together.

```bash
roslaunch phyto_gazebo winch.launch
```

The payload starts suspended with the motor stopped. In a second sourced
terminal, send a goal to lower it to 2.20 m at up to 0.18 m/s. Send another
goal with `depth: 0.85` to bring it back up.

```bash
rostopic pub -1 /winch/move_to_depth/goal phyto_arm/MoveToDepthActionGoal \
    '{goal: {depth: 2.20, velocity: 0.18}}'
rostopic echo -n 1 /ctd/depth
```

Pass `gui:=false` to run headless. From a development workspace, exercise a
descent, encoder zero and recovery with the integration test.

```bash
rostest phyto_gazebo gazebo_winch.test
```

## Try the tether

To command the paid-out length directly, launch just the world.

```bash
roslaunch gazebo_ros empty_world.launch \
    world_name:="$(rospack find phyto_gazebo)/worlds/winch.world"
```

In a second sourced terminal, pay out the line to 3.15 m. Give the payload
time to settle, then haul back to the starting length of 1.32 m.

```bash
rostopic pub -1 /sim/line_length std_msgs/Float64 'data: 3.15'
rostopic echo -n 1 /sim/ctd_position
rostopic pub -1 /sim/line_length std_msgs/Float64 'data: 1.32'
```

`/sim/ctd_position` gives the simulated probe position in metres in the world
frame. The water surface is at `z = 0` and depth increases downward.

To drive the line with the simulated motor, start
`roslaunch jvl_motor sim_motor.launch` and `rosrun phyto_sim_utils winch_drum.py`
in separate sourced terminals. The drum uses the 60:1 gearbox and 0.6604 m
circumference from `configs/example.yaml`. Zeroing the motor encoder leaves
the paid-out length unchanged.

## Read the CTD

The water-column model publishes `/sim/ctd` and `/sim/depth` at the Gazebo
observation time. The AML emulator pairs those samples and sends checksummed
AMLx monitor records on `/sim/aml`. The production AML driver publishes
`/ctd/ctd`, `/ctd/depth` and the individual measurements. Read salinity on
`/ctd/aml/derive/sal_c`. The winch uses the driver's depth in metres.

```bash
rostopic echo -n 1 /ctd/depth
rostopic echo -n 1 /ctd/ctd
```

## Read the GPS

The barge GPS publishes its Gazebo position on `/sim/gps` once per simulated
second. The world origin is at 41.524° N, 70.672° W near Woods Hole. The GPS
emulator sends NMEA RMC and GGA sentences over UDP to a real `gpsd`. Its TCP
client port and UDP input port are both 29470 (set with `gpsd_port`). The
production `gpsd_client` publishes `/gps/fix` and `/gps/extended_fix` using
ROS time.
Receiver UTC starts at the time of the first fix and advances with Gazebo.

## Sample with a lightweight IFCB

Pass `mock_ifcb:=true` to `winch.launch`, or run
`roslaunch ifcb ifcb_mock.launch` for the IFCB alone. The mock exposes the
production command and routine services, connection status, raw messages,
images and ROI topics. The skeletal sample routine releases the winch at the
sampling-speed marker and emits a one-pixel test image. Waits use ROS time.

For camera, serial and acquisition-file tests, run IFCBacquire with
ifcb-simulator.

## Switch an outlet

The production DLI driver polls an eight-outlet HTTP simulator on loopback
port 18080 (configurable with `dli_port`). The launch configures the Starlink
outlet, initially on. Switching it changes its reported state. The
simulated instruments keep running.

```bash
rostopic pub -1 /digital_logger/outlet/starlink/control std_msgs/Bool 'data: false'
rostopic echo -n 1 /digital_logger/outlet/starlink/status
```

## Physical assumptions

The line is a massless spring and damper that can pull on the lifting eye.
Gravity, buoyancy, drag and a steady current act on the payload. The housing
dimensions come from the McLane IFCB datasheet. The combined payload mass,
displacement and drag are estimates. The barge and water surface are fixed.
