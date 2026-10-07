# Quickstart — Ubuntu 24.04 / ROS 2 Jazzy

## 1. Move the workspace to Linux

If you use WSL, copy the directory to a location on the Linux filesystem such as `~/arm_o7_digital_twin`. Gazebo mesh loading and `colcon` operations can be slower under `/mnt/c`.

```bash
cd ~/arm_o7_digital_twin
bash scripts/install_dependencies.sh
bash scripts/build.sh
```

`build.sh` runs `rosdep`, `colcon build`, the project validator, and package tests.

## 2. Terminal 1 — Gazebo

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal1_gazebo.sh
```

After Gazebo opens and the world starts, move to the second terminal.

## 3. Terminal 2 — robot + MoveIt

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal2_robot.sh
```

Expected controllers:

```bash
ros2 control list_controllers -c /sim/controller_manager
```

All four must be `active` in the expected result:

```text
joint_state_broadcaster
arm_controller
hand_controller
thumb_coupling_controller
```

Additional checks:

```bash
ros2 control list_hardware_interfaces -c /sim/controller_manager
ros2 topic hz /sim/joint_states
ros2 action list | grep follow_joint_trajectory
ros2 topic echo --once /twin/status
```

## 4. Simulation-only motion test

Open a third shell, or run the launch in `tmux` in Terminal 2:

```bash
cd ~/arm_o7_digital_twin
bash scripts/demo_sim_motion.sh
```

Alternatively, select the `arm` group in RViz's MotionPlanning panel and plan and execute it. For the `hand` group, use the collision-free named states `open`, `pregrasp`, `four_finger_fist`, and `middle_finger`. `four_finger_fist` curls the other four fingers fully through their PIP/DIP mimic joints while the thumb remains open; with suitable arm/wrist orientation it forms the hand portion of a thumbs-up sign. In `middle_finger`, the middle finger is open, the other three long fingers are closed, and the thumb is safely open. `contact_closed` represents fingertip contact and must not be used directly as an OMPL target.

To demonstrate the prepared hand gestures in sequence from Terminal 3:

```bash
cd ~/arm_o7_digital_twin
bash scripts/demo_hand_gestures.sh
```

The expected final line is `HAND_GESTURE_DEMO=PASS`. Motions appear in both Gazebo and RViz, and the script uses only the `/sim` action.

Qualification test covering all arm/hand named states through MoveIt:

```bash
bash scripts/run_qualification.sh 1 2>&1 | tee qualification.log
```

The expected final line is `DIGITAL_TWIN_QUALIFICATION=PASS`.

## 5. Set the mounting transform

If the real O7 adapter has an offset or yaw, add it to the Terminal 2 command in meters/radians:

```bash
bash scripts/terminal2_robot.sh \
  hand_mount_xyz:="0 0 0.012" \
  hand_mount_rpy:="0 0 1.57079632679"
```

These two values are applied to both the Gazebo and MoveIt models. Do not command physical motion using guessed values that have not been measured.

## 6. Troubleshooting

If `/sim/controller_manager` cannot be found:

```bash
gz sim --versions
ros2 pkg prefix gz_ros2_control
ros2 topic echo --once /sim/robot_description
```

If meshes are not visible:

```bash
ros2 pkg prefix arm_o7_description
python3 tests/validate_project.py
```

If Gazebo opens but the robot does not move, check the controller log for `gz_ros2_control/GazeboSimSystem` and confirm that all four controllers are `active`.

If WSL has GUI problems, verify the WSLg/OpenGL installation; it is easier to complete controller and topic tests with a headless world first.
