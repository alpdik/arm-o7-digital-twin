# Team setup: ARM1.5 + LinkerHand O7 starter package

This package contains the verified combined digital-twin sources for the **right LinkerHand O7** and ARM1.5 arm. It does not include a ready-made `build/` or `install/` directory because those must be regenerated on each computer. Physical robot output is disabled by default, and the initial goal is simulation only.

The target system is Ubuntu 24.04 and ROS 2 Jazzy. `install_dependencies.sh` assumes that the official ROS 2 apt repository and `/opt/ros/jazzy` installation are ready; it does not install ROS from scratch. If you extract the ZIP from the command line and `unzip` is missing, run `sudo apt install unzip` first or use Ubuntu's archive manager.

## 1. Extract the ZIP on the Linux side

If you use WSL2, adjust the ZIP path in the Windows `Downloads` directory for your Windows username:

```bash
cd ~
unzip /mnt/c/Users/<WINDOWS_KULLANICI_ADI>/Downloads/ARM1_5_O7_TEAM_STARTER_2026-10-07_v3.zip
cd ~/arm_o7_digital_twin
chmod +x scripts/*.sh scripts/*.py
```

On native Ubuntu, extract the ZIP directly into your home directory and enter `~/arm_o7_digital_twin` again. Do not build the project under `/mnt/c` in WSL; the Linux home directory is faster and more reliable.

## 2. Collect the computer report first

This step installs no packages and changes no system settings:

```bash
cd ~/arm_o7_digital_twin
bash scripts/preflight_report.sh
```

Send the resulting `preflight_report.txt` file to your teammate or AI assistant. Check these items in particular:

- Ubuntu version: target 24.04
- ROS 2: target Jazzy
- WSL2/WSLg or native Ubuntu
- CPU core count and RAM
- OpenGL `renderer` and `Accelerated: yes/no`
- NVIDIA, AMD, Intel, or software rendering status

Do not assume the GPU brand in advance. On WSLg, the scripts use the D3D12 driver and automatically select the adapter through Mesa. If the report shows multiple GPUs and selects the wrong one, use for example:

```bash
export ARM_O7_GPU_ADAPTER=NVIDIA
```

For Intel or AMD, the value can be a distinctive part of the relevant card name. If `llvmpipe` appears, inspect the report before forcing a GPU.

## 3. Install and build

When the preflight check is suitable:

```bash
cd ~/arm_o7_digital_twin
bash scripts/install_dependencies.sh
bash scripts/build.sh 2>&1 | tee build.log
```

Expected key results:

```text
Summary: 5 packages finished
Summary: 28 tests, 0 errors, 0 failures, 0 skipped
```

If the numbers or results differ, share the last 150 lines of `build.log` before continuing:

```bash
tail -n 150 build.log
```

## 4. Start the simulation in two terminals

Terminal 1:

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal1_gazebo.sh 2>&1 | tee terminal1_gazebo.log
```

After the Gazebo world window opens, use Terminal 2:

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal2_robot.sh 2>&1 | tee terminal2_robot.log
```

On a weak computer, MoveIt/RViz can be temporarily disabled for the first controller test:

```bash
ARM_O7_START_MOVEIT=false bash scripts/terminal2_robot.sh \
  2>&1 | tee terminal2_robot_light.log
```

This lightweight mode is for diagnostics only. To reach the full target, later start MoveIt and RViz with the normal Terminal 2 command.

If using VirtualBox, enable 3D acceleration in the virtual machine display settings and allocate as much CPU/RAM as possible. Do not guess the hardware brand; check the actual renderer in the `OpenGL renderer` line of `preflight_report.txt`. `llvmpipe` indicates software rendering.

## 5. Check and move from a third terminal

```bash
cd ~/arm_o7_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 control list_controllers -c /sim/controller_manager
ros2 action list -t | grep follow_joint_trajectory
bash scripts/demo_combined_motion.sh 2>&1 | tee combined_motion_fist.log
bash scripts/demo_hand_gestures.sh 2>&1 | tee hand_gestures.log
```

All expected controllers must be `active`:

```text
joint_state_broadcaster
arm_controller
hand_controller
thumb_coupling_controller
```

Expected motion-test result:

```text
[outbound] ARM=SUCCEEDED HAND=SUCCEEDED
[return] ARM=SUCCEEDED HAND=SUCCEEDED
COMBINED_SIM_TEST=PASS
HAND_GESTURE_DEMO=PASS
```

Motions must appear together in Gazebo and RViz. The demos use only `/sim` actions and do not publish to `/real`. The gesture demo opens the hand, shows `middle_finger`, opens it again, makes a four-finger fist, and returns the hand to the open position.

Then run the qualification test, which also covers MoveIt planning and execution:

```bash
bash scripts/run_qualification.sh 1 2>&1 | tee qualification.log
```

Expected final line:

```text
DIGITAL_TWIN_QUALIFICATION=PASS
```

JSON and CSV evidence is created under `results/digital_twin_qualification/`. After the first cycle passes, repeat the same test for `10` cycles if desired.

## Shared checkpoint to reach

When setup is complete, the teammate's system should be in this state:

1. ARM1.5 and right O7 appear as one robot in Gazebo.
2. The same robot appears in the RViz MotionPlanning view.
3. Four controllers are active.
4. Arm and hand action servers are listed.
5. Arm and hand move out and back together; the combined demo returns `PASS`.
6. RViz named states provide `home`/`ready`/`left_demo` for the arm and `open`, `pregrasp`, `four_finger_fist`, and `middle_finger` for the hand.
7. `run_qualification.sh 1` ends with `DIGITAL_TWIN_QUALIFICATION=PASS`.

## For the person continuing with an AI assistant

Paste the contents of `AI_HANDOFF_PROMPT.txt` from the ZIP as the first message in a new conversation, and attach the ZIP followed by `preflight_report.txt`. Ask the assistant to verify and run this package rather than regenerate the model. This reduces the risk of rebuilding the O7 integration from scratch in a different form. This file is not tied to a specific AI product.

## License note

LinkerHand O7 assets are provided under the Apache-2.0 license. No license was found in the ARM1.5 mesh/URDF archive. This ZIP is for internal technical transfer; verify permission/license status with the rights holder before publishing ARM1.5 files in a public repository or distributing them outside the team. Details are in `LICENSES/ARM_ASSETS_NOTICE.txt`.
