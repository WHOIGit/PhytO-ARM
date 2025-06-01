---
title: "Operating winch"
weight: 2
---

DRAFT

Once the networking has been set up and all requites softwares have been installed you are ready to begin controlling your PhytO-Arm system. First log into your IFCB via ssh tunnels. Launch the PhytO-arm software (HOW do you do this?).

#### First, prepare your session for interacting with ROS:

```{bash include = TRUE}
cd ~/PhytO-ARM source devel/setup.bash
```

#### To STOP the motor, issue a command like this:

```{bash}
rosservice call /motor_aft/stop
#replace motor_aft with the ros node name of your PhytO-Arm system
```

#### To command the motor to move to a given position, use:

```{bash}
rosservice call /motor_aft/set_position "{ position: <X>, velocity: <V>, acceleration: 1200.0, torque: 3.0 }"
#Substitute <X> with the target encoder count and <V> with the velocity in RPM.

```

**Velocity should be encoded as negative for upward motion and positive for downward motion**. Velocities on the order of 10000 will result in an estimated rate of 2 cm/s

If unsure about direction, before moving the motor, you can get the current encoder position. This can be found in Foxglove Studio (e.g., using a Raw Messages panel looking at the /motor_aft/motion topic for the "position" field) or by running a command:

```{bash}
rostopic echo -n 1 /motor_aft/motion
```

We recommend establishing a table of encoder position to depth mappings. (DO WE HAVE THIS TABLE? COULD WE SHARE?). You can set a safety envelope on the motor position so that the motor will refuse to exceed the bounds and thus not put your instruments into the bottom or pull them out of the water:

```{bash}
rosservice call /motor_aft/set_position_envelope "{ min: <X>, max: <Y> }"
```

## The calculation for converting a distance (in meters) to encoder counts is:

8192 \* gear_ratio \* distance / spool_circumference

where gear_ratio = 60 and we don't yet know the spool_circumference but we can derive it from your table.

8192 is the number of encoder counts to turn the motor 360°
