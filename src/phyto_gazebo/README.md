# Gazebo winch simulator

The world contains a fixed barge with a winch and a suspended IFCB and CTD.
Gazebo applies tension at the lifting eye when the line is stretched. Paying
out extra line lets the payload fall until the line catches it again.

## Try the tether

From a sourced ROS Noetic workspace with Gazebo Classic 11, start the world.
Add `gui:=false` to run headless.

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

## Physical assumptions

The line is a massless spring and damper that can pull on the lifting eye.
Gravity, buoyancy, drag and a steady current act on the payload. The housing
dimensions come from the McLane IFCB datasheet. The combined payload mass,
displacement and drag are estimates. The barge and water surface are fixed.
