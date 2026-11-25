# Simulating a PhytO-ARM deployment

PhytO-ARM supports simulation of a winch, CTD, GPS receiver and IFCB without connecting any hardware. From a built ROS Noetic workspace, source `devel/setup.bash` in each terminal and start the simulator:

```bash
roslaunch phyto_arm simulator.launch
```

With the defaults, the arm runs automatic casts and samples with a mock IFCB. Append `option:=value` to the command to change the deployment.

| Option | Default | What it controls |
| --- | --- | --- |
| `gazebo` | `false` | Set to `true` to command the winch in a Gazebo world. |
| `gui` | `true` | Shows the Gazebo 3D view. Set to `false` for headless Gazebo. |
| `gpsd` | `false` | Set to `true` to send simulated NMEA sentences through an installed `gpsd` daemon and the production GPS client. |
| `mock_ifcb` | `true` | Set to `false` to run IFCBacquire with a simulated camera and serial ports. |
| `config_file` | `configs/example.yaml` | Selects the deployment configuration for the lightweight arm. |
| `ifcb_replay_dir` | `~/ifcb-replay` | Locates the lab checkout used when `mock_ifcb:=false`. |

## Lightweight arm

The arm profiles the water column, chooses a sampling depth, and holds there through sample intake. In another terminal, watch `rostopic echo /arm_ifcb/task` for `winch_movement`, or run `rostopic echo /arm_ifcb/ctd/depth/depth` to follow the depth in metres.

## Gazebo

With Gazebo Classic 11 installed and `phyto_gazebo` built in your workspace, start the simulator in Gazebo mode:

```bash
roslaunch phyto_arm simulator.launch gazebo:=true
```

A 3D view shows the CTD and IFCB hanging from a fixed barge. The motor starts stopped. In a second terminal, lower the payload to 2.2 m:

```bash
rostopic pub -1 /winch/move_to_depth/goal phyto_arm/MoveToDepthActionGoal \
    '{goal: {depth: 2.2, velocity: 0.18}}'
rostopic echo /ctd/depth/depth
```

The reported depth should approach 2.2 m. Press Ctrl-C to stop the readout, then take a sample with the mock IFCB:

```bash
rosservice call /ifcb/routine '{routine: runsample, instrument: true}'
```

To raise the payload, send the winch goal again with `depth: 0.85`.

## How the instruments are simulated

### Winch and CTD

Both deployments run the production winch controller and motor driver against a simulated MAC400. Motor rotation pays line off the drum or takes it up. The lightweight arm assumes a taut vertical line. In Gazebo the payload can swing in a steady current or fall when the line goes slack. The water surface is fixed, and payload mass and drag are estimates.

Both CTDs sample the same water column at the probe's depth. Pressure increases with depth and the surface layer is warmer. The lightweight arm publishes depth and water properties directly on `/arm_ifcb/ctd/depth` and `/arm_ifcb/ctd/ctd`. Gazebo passes those readings through an AML emulator. The production AML driver decodes its monitor records and publishes `/ctd/depth` and `/ctd/ctd`.

### GPS

By default, `gpsd_client_mock` publishes `/gps/fix` and `/gps/extended_fix` directly. The lightweight arm uses a stationary position and Gazebo supplies the barge antenna position. With `gpsd:=true`, a simulated receiver encodes that position as NMEA sentences, feeds them to `gpsd`, and the production `gpsd_client` publishes the same two topics.

### IFCB

The mock provides the same ROS command and routine services as the IFCB driver. Its short routines signal when intake is complete and emit a placeholder image. This is enough to test when the arm can resume moving. To test camera frames, serial communication or acquisition files, follow these steps to take a sample with the original IFCBacquire software.

These steps require Docker on the Linux ROS host and lab access to both the IFCB simulator and the `ifcb/ifcbacquire:3.0.0` image. For ROS sampling, use the lab's `ifcb-replay` checkout containing `ros/run.sh`. The [private ifcb-simulator repository](https://github.com/WHOIGit/ifcb-simulator) includes the standalone replay, but its `main` branch currently lacks the ROS launcher.

1. **Build the camera image.** From the `ifcb-replay` checkout, build the simulated GigE camera:

   ```bash
   docker build -t phyto/aravis-ifcb:local camera
   ```

2. **Start a deployment with IFCBacquire.** This example starts Gazebo with the camera, emulated DAQ and fluidics ports, and production IFCB driver:

   ```bash
   roslaunch phyto_arm simulator.launch gazebo:=true mock_ifcb:=false
   ```

   The lab launcher disables scheduled maintenance routines and the post-sample flush for these sessions.

3. **Request a sample.** The lightweight arm requests its own samples. In Gazebo, wait for the connection and send a goal from another terminal:

   ```bash
   rostopic echo -n 1 --filter 'm.data' /ifcb/connected
   rostopic pub -1 /arm_ifcb/ifcb_runner/sample/goal \
       phyto_arm/RunIFCBActionGoal '{}'
   ```

   Look under `data/testing/` in the session directory printed by the launcher for the resulting `.hdr`, `.adc` and `.roi` files.

## Testing individual components

**Motor.** To command the simulated shaft directly or test encoder limits, see the [MAC400 driver examples](src/jvl_motor/README.md).

**Tether.** To control the line length directly, launch just the world on a fresh ROS master:

```bash
roslaunch gazebo_ros empty_world.launch \
    world_name:="$(rospack find phyto_gazebo)/worlds/winch.world"
```

In another terminal, pay out line, read the probe position, then haul it back. The position is in metres, with the water surface at `z = 0`.

```bash
rostopic pub -1 /sim/line_length std_msgs/Float64 'data: 3.15'
rostopic echo -n 1 /sim/ctd_position
rostopic pub -1 /sim/line_length std_msgs/Float64 'data: 1.32'
```

To test the whole winch against depth goals and an encoder reset, run `rostest phyto_gazebo gazebo_winch.test` in a sourced workspace.

**Power switch.** The Gazebo deployment includes the production DLI driver and a simulated outlet. Switching it changes the reported state. The instruments keep running.

```bash
rostopic pub -1 /digital_logger/outlet/starlink/control std_msgs/Bool \
    'data: false'
rostopic echo -n 1 /digital_logger/outlet/starlink/status
```

**IFCB.** To run just the mock services, use `roslaunch ifcb ifcb_mock.launch`.
