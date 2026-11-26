# Simulating a PhytO-ARM deployment

PhytO-ARM supports simulation of a winch, CTD, GPS receiver and IFCB without connecting any hardware. From a built ROS 2 Humble workspace, source `install/setup.bash` in each terminal and start the simulator:

```bash
ros2 launch phyto_arm simulator.launch.py
```

With the defaults, the arm runs automatic casts and samples with a mock IFCB. Append `option:=value` to the command to change the deployment.

| Option | Default | What it controls |
| --- | --- | --- |
| `gazebo` | `false` | Set to `true` to command the winch in a Gazebo world. |
| `ctd` | `direct` | Publishes water-column readings directly. Select `aml` or `rbr` to run the matching emulator and production CTD driver. |
| `gui` | `true` | Shows the Gazebo 3D view. Set to `false` for headless Gazebo. |
| `gpsd` | `false` | Set to `true` to send simulated NMEA sentences through an installed `gpsd` daemon and the production GPS client. |
| `mock_ifcb` | `true` | Set to `false` to run IFCBacquire with a simulated camera and serial ports. |
| `config_file` | `configs/example.yaml` | Selects the deployment configuration for the lightweight arm. |
| `ifcb_replay_dir` | `~/ifcb-replay` | Locates the lab checkout used when `mock_ifcb:=false`. |

## Lightweight arm

The arm profiles the water column, chooses a sampling depth, and holds there through sample intake. In another terminal, watch `ros2 topic echo /arm_ifcb/task` for `winch_movement`, or run `ros2 topic echo /arm_ifcb/ctd/depth --field depth` to follow the depth in metres.

## Gazebo

With Gazebo Fortress installed and `phyto_gazebo` built in your workspace, start the simulator in Gazebo mode:

```bash
ros2 launch phyto_arm simulator.launch.py gazebo:=true
```

A 3D view shows the CTD and IFCB hanging from a fixed barge. The motor starts stopped. In a second terminal, lower the payload to 2.2 m:

```bash
ros2 action send_goal /winch/move_to_depth phyto_arm_msgs/action/MoveToDepth \
    '{depth: 2.2, velocity: 0.18}'
ros2 topic echo /ctd/depth --field depth
```

The reported depth should approach 2.2 m. Press Ctrl-C to stop the readout, then take a sample with the mock IFCB:

```bash
ros2 service call /ifcb/routine ifcb_msgs/srv/RunRoutine '{routine: runsample, instrument: true}'
```

To raise the payload, send the winch goal again with `depth: 0.85`.

## How the instruments are simulated

### Winch and CTD

Both deployments run the production winch controller and motor driver against a simulated MAC400. Motor rotation pays line off the drum or takes it up. The lightweight arm assumes a taut vertical line. In Gazebo the payload can swing in a steady current or fall when the line goes slack. The water surface is fixed, and payload mass and drag are estimates.

Both deployments sample the same water column at the probe's depth and publish the model readings directly by default. Pressure increases with depth and the surface layer is warmer. To exercise the RBR Maestro3 driver in either deployment, add `ctd:=rbr` to `ros2 launch phyto_arm simulator.launch.py`.

The AML emulator sends monitor records on `sim/aml`. The RBR emulator sends timestamped CSV records on `sim/rbr`. Both drivers publish `ctd/depth` and `ctd/ctd` under `/arm_ifcb` in the lightweight deployment and at the root in Gazebo. The RBR driver reports conductivity in mS/cm on `ctd/rbr/conductivity` and in S/m on `ctd/ctd`.

### GPS

By default, `gpsd_client_mock` publishes `/gps/fix` and `/gps/extended_fix` directly. The lightweight arm uses a stationary position and Gazebo supplies the barge antenna position. With `gpsd:=true`, a simulated receiver encodes that position as NMEA sentences, feeds them to `gpsd`, and the production `gpsd_client` publishes the same two topics.

### IFCB

The mock provides the same ROS command and routine services as the IFCB driver. Its short routines signal when intake is complete and emit a placeholder image. This is enough to test when the arm can resume moving. To test camera frames, serial communication or acquisition files, follow these steps to take a sample with the original IFCBacquire software.

These steps require Docker on the Linux ROS host and lab access to both the IFCB simulator and the `ifcb/ifcbacquire:3.0.0` image. For ROS sampling, use the lab's `ifcb-replay` checkout containing the camera, fixture and acquisition tools. The [private ifcb-simulator repository](https://github.com/WHOIGit/ifcb-simulator) includes the standalone replay, but its `main` branch currently lacks the ROS launcher.

1. **Build the camera image.** From the `ifcb-replay` checkout, build the simulated GigE camera:

   ```bash
   docker build -t phyto/aravis-ifcb:local camera
   ```

2. **Start a deployment with IFCBacquire.** This example starts Gazebo with the camera, emulated DAQ and fluidics ports, and production IFCB driver:

   ```bash
   ros2 launch phyto_arm simulator.launch.py gazebo:=true mock_ifcb:=false
   ```

   The emulator launcher disables scheduled maintenance routines and the post-sample flush for these sessions.

3. **Request a sample.** The lightweight arm requests its own samples. In Gazebo, wait for the connection and send a goal from another terminal:

   ```bash
   ros2 topic echo /ifcb/connected --once --qos-durability transient_local --filter 'm.data'
   ros2 action send_goal /arm_ifcb/ifcb_runner/sample \
       phyto_arm_msgs/action/RunIFCB '{}'
   ```

   Look under `data/testing/` in the session directory printed by the launcher for the resulting `.hdr`, `.adc` and `.roi` files.

## Testing individual components

**Motor.** To command the simulated shaft directly or test encoder limits, see the [MAC400 driver examples](src/jvl_motor/README.md).

**Tether.** To control the line length directly, launch just the world in a separate ROS domain:

```bash
ign gazebo -s -r "$(ros2 pkg prefix --share phyto_gazebo)/worlds/winch.world"
```

In another terminal, pay out line, read the probe position, then haul it back. The position is in metres, with the water surface at `z = 0`.

```bash
ros2 topic pub --once --qos-durability transient_local --keep-alive 3 /sim/line_length std_msgs/msg/Float64 'data: 3.15'
ros2 topic echo /sim/ctd_position --once
ros2 topic pub --once --qos-durability transient_local --keep-alive 3 /sim/line_length std_msgs/msg/Float64 'data: 1.32'
```

To test the whole winch against depth goals and an encoder reset, run `colcon test --packages-select phyto_gazebo --ctest-args -R gazebo_winch` in a sourced workspace.

**Power switch.** The Gazebo deployment includes the production DLI driver and a simulated outlet. Switching it changes the reported state. The instruments keep running.

```bash
ros2 topic pub --once /digital_logger/outlet/starlink/control std_msgs/msg/Bool \
    'data: false'
ros2 topic echo /digital_logger/outlet/starlink/status
```

**IFCB.** To run just the mock services, use `ros2 launch ifcb ifcb_mock.launch.py`.
