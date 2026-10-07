# LinkerHand O7 SDK adapter

This package bridges the canonical seven actuated O7 joints to the official
LinkerHand `sensor_msgs/JointState` radian topics. It also merges separately owned
physical arm and hand state streams into the complete state expected by
`arm_o7_twin`.

It starts fail-closed. No SDK command can be published unless all of the following
are true: `command_enabled`, `mapping_verified`, live deadman, live heartbeat, and
a fresh SDK state. Output is rate-limited to 30 Hz and trajectories are validated,
bounded, and interpolated.

The checked-in right-hand mapping is provisional. It combines the current O7 URDF
limits with the official SDK's documented L7-R arc order/ranges; upstream itself
warns that the right-hand URDF will change. Read-only state observation and
joint-by-joint physical calibration are required before enabling commands.

```bash
ros2 launch arm_o7_linkerhand_adapter hardware_adapter.launch.py
```

Physical arm state must be published or remapped to `/real/arm_joint_states`.
The O7 adapter owns `/real/hand_joint_states`; the mux alone owns the combined
`/real/joint_states` topic.
