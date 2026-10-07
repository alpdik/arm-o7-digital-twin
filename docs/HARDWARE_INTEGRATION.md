# Physical hardware integration

## ARM1.5 arm

The supplied archive contained only CAD URDF/STL files. It did not include a vendor driver, encoder protocol, motor control mode, transmission, or safety limits. Therefore, no `ros2_control` plugin was invented for the physical arm.

The real backend must provide this contract:

- `/real/arm_controller/follow_joint_trajectory`
- `/real/arm_controller/joint_trajectory`
- `/real/arm_joint_states` (six arm joints only)
- Joint names `arm_joint_1` … `arm_joint_6`

Source mapping:

| CAD joint | Canonical joint | Axis | Position range (rad) |
|---|---|---|---|
| A | `arm_joint_1` | Z | -3.14 … 3.14 |
| B | `arm_joint_2` | X | -1.57 … 1.57 |
| C | `arm_joint_3` | X | -1.30 … 1.30 |
| D | `arm_joint_4` | Z | -3.14 … 3.14 |
| E | `arm_joint_5` | X | -1.30 … 1.30 |
| F | `arm_joint_6` | Z | -3.14 … 3.14 |

Do not use this mapping for commands until encoder direction/zero is verified by physical measurement.

## LinkerHand O7

The official `linker-bot/linkerhand-ros2-sdk` lists the O7 and its radian (`*_arc`) command/state topics. However, the environment stated by the upstream README is Ubuntu 22.04 + ROS 2 Humble; Jazzy support has not been verified upstream.

`arm_o7_linkerhand_adapter` does the following:

- Converts the canonical 7-joint order to the SDK order.
- Applies sign/scale/offset when required.
- Samples output at the official SDK's stated maximum of 30 Hz.
- Converts SDK state back to `/real/hand_joint_states` format.
- Combines fresh arm and hand state and publishes the `/real/joint_states` topic, for which it is the sole owner.
- Starts with `mapping_verified=false`, enable/deadman disabled, and a timeout.

Startup:

```bash
source ~/arm_o7_digital_twin/install/setup.bash
ros2 launch arm_o7_linkerhand_adapter hardware_adapter.launch.py
```

If your arm driver publishes `/joint_states`, remap it to `/real/arm_joint_states` in the launch/remap configuration. Do not allow the arm driver, hand adapter, and mux all to publish `/real/joint_states`.

The delivered right-hand mapping is **provisional**: O7 URDF limits were normalized to the L7-R arc order/ranges in the official SDK. Upstream documentation also states that the right-hand URDF will change. Do not enable `mapping_verified` or `command_enabled` until the index, scale, and offset values in `config/o7_right.yaml` are verified against the actual O7 model/firmware and single-joint measurements.

CAN enablement, system passwords, and `sudo` operations are not the adapter's responsibility. Manage them separately through the operating system/udev and the manufacturer's installation procedure.

## Low-speed commissioning sequence

1. With the robot powered off, measure mechanical limits, directions, and the hand adapter.
2. Test the physical E-stop and driver watchdog independently.
3. Read state only; compare canonical joint names/directions/zeros against the record.
4. Observe physical motion in Gazebo in `SHADOW` mode; keep real output disabled.
5. Test each joint individually with small steps and low speed.
6. Propagate actual velocity/acceleration/effort limits from the same source file to the URDF, MoveIt, controller, and arbiter layers.
7. Tighten startup and tracking-error tolerances based on measured latency/noise.
8. Only after risk assessment, enable `enable_real_output` in the arbiter and `mapping_verified` and `command_enabled` in the O7 adapter.

Once the real controller accepts a trajectory, blocking new commands at the topic-based arbiter does not necessarily cancel that trajectory. Production use requires a downstream `FollowJointTrajectory` action proxy/cancel flow and a driver-level quick stop.
