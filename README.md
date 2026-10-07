# ARM1.5 + LinkerHand O7 digital twin starter project

This workspace combines the user-supplied six-axis ARM1.5 model with the **right O7** model from the `linker-bot/linkerhand-urdf` repository as one ROS 2 robot. The target environment is Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, `gz_ros2_control`, and MoveIt 2.

This delivery includes a working simulation/planning foundation and a fail-closed digital-twin command arbiter. Because the physical arm driver was not included in the archive, command output to the real robot is disabled by default. The right/left O7 choice must also be confirmed; right O7 was assumed from the photograph.

## Included

- 6 ARM + 7 independent O7 actuators; 10 mechanically linked O7 joints use `mimic`
- Corrected mesh URIs, unique link/joint names, and a ground-fixed robot root
- `gz_ros2_control` and two trajectory controllers for Gazebo Harmonic
- MoveIt groups: `arm`, `hand`, `arm_with_hand`
- Much lighter convex collision meshes derived from the raw visual meshes
- Safety modes: `SAFE_IDLE`, `SIM_ONLY`, `SHADOW`, `TWIN_COMMAND`, `REAL_ONLY`
- Heartbeat, deadman, stale-state, startup-error, tracking-error, and joint-limit interlocks
- Separate, disabled-by-default adapter package for the real O7 SDK
- A mux that combines separate physical arm/hand state into the single-owner `/real/joint_states` stream
- Static validation and pure-logic tests that run without a ROS installation

## Packages

| Package | Purpose |
|---|---|
| `arm_o7_description` | Combined Xacro/URDF, original and collision meshes, ros2_control definition |
| `arm_o7_bringup` | Gazebo world, spawn, controller sequencing, and two-terminal launch flow |
| `arm_o7_moveit_config` | SRDF, KDL, OMPL, joint limits, controller, and RViz settings |
| `arm_o7_twin` | Sim/real command arbiter, SHADOW mirroring, and safety interlocks |
| `arm_o7_linkerhand_adapter` | Adapter between the official O7 ROS 2 SDK topics and the canonical hand interface |
| `arm_o7_glove` | Drive the O7 hand in simulation with a 5DT data glove and forward to the real Ti5 hand |

## Initial setup

Copying the project to the Linux home directory in WSL is faster and more reliable than building under `/mnt/c`:

```bash
cd ~/arm_o7_digital_twin
bash scripts/install_dependencies.sh
bash scripts/build.sh
```

For quick use, follow [QUICKSTART.md](QUICKSTART.md). To install the project on another team computer, first follow [TEAM_START_HERE.md](TEAM_START_HERE.md). The package is not tied to a hardware brand; it automatically selects the WSLg graphics adapter, and `ARM_O7_GPU_ADAPTER` can be used when necessary.

## Two Ubuntu terminals

Terminal 1 — Gazebo world server, GUI, and one-way `/clock` bridge:

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal1_gazebo.sh
```

Terminal 2 — spawn the robot, start controllers in order, and launch MoveIt/RViz and the safety arbiter:

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal2_robot.sh
```

Simulation validation motion:

```bash
cd ~/arm_o7_digital_twin
bash scripts/demo_sim_motion.sh
```

This demo uses only `/sim/...` actions and produces no `/real/...` output.

To demonstrate the `open`, `middle_finger`, and `four_finger_fist` hand gestures in sequence:

```bash
bash scripts/demo_hand_gestures.sh
```

The expected final line is `HAND_GESTURE_DEMO=PASS`.

Local digital-twin qualification test after the first motion:

```bash
cd ~/arm_o7_digital_twin
bash scripts/run_qualification.sh 1 2>&1 | tee qualification.log
```

The test measures controllers and action servers, MoveIt state-validity results, `/joint_states` rate, Gazebo real-time factor, MoveIt planning/execution, and final joint errors together. It uses the `home`, `ready`, `left_demo`, `open`, `pregrasp`, `four_finger_fist`, and `middle_finger` motions. JSON and CSV evidence is written to a dated directory under `results/digital_twin_qualification/`. After the single cycle passes, run the ten-cycle regression:

```bash
bash scripts/run_qualification.sh 10 2>&1 | tee qualification_10_cycles.log
```

## Glove control (5DT glove + Ti5 hand)

With Terminal 1 and Terminal 2 open, use a third terminal:

```bash
cd ~/arm_o7_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch arm_o7_glove glove_teleop.launch.py
```

In the window that opens, first calibrate the glove with **Calibrate glove** (open hand, then fist); this is required at every startup. **ON** then drives the simulated hand with the glove. If **Also move the REAL Ti5 hand** is checked, the real Ti5 hand also follows the glove. Device permissions, the required udev rule, mapping, and safety details are in [src/arm_o7_glove/README.md](src/arm_o7_glove/README.md).

## Design boundary: simulation and real hardware

```text
Glove / operator UI / approved plan
                 |
          /twin/joint_trajectory
                 |
       fail-closed komut hakemi
          /                 \
 /sim/*_controller    /real/*_controller
          \                 /
       sim ve real joint_states
                 |
        tracking-error observer
```

Real and sim joint-state topics are not connected directly to each other's command topic. In `SHADOW` mode, real state is reflected only into simulation. In `TWIN_COMMAND` mode, the same validated reference is split to both backends and the results are also compared. This prevents bidirectional data flow from becoming a positive-feedback loop.

In this first delivery, MoveIt/RViz directly executes `/sim` action controllers. Physical execution is deliberately routed through the arbiter topic; a traceable and cancellable downstream action proxy for MoveIt should be added in a second phase after the real arm driver is known.

## Deliberately disabled

- `enable_real_output` defaults to `false`
- `mapping_verified` in the O7 adapter defaults to `false`
- `command_enabled` in the O7 adapter defaults to `false`
- No fake driver is provided for the physical arm
- Velocity/effort values not supplied by the manufacturer are not treated as physical limits
- A ROS node is not trusted alone on network loss; a driver watchdog and physical E-stop are required

## Measurements/information required for the next phase

1. Arm make/model, motor drivers, and communication protocol (CAN, EtherCAT, Modbus/TCP, serial, etc.).
2. Physical joint order, positive directions, encoder zeros, and velocity/acceleration/effort limits.
3. Whether O7 is right or left, firmware/SDK version, and CAN/RS485 selection.
4. `arm_flange -> hand_base_link` adapter thickness and RPY transform. The initial assumption is `xyz="0 0 0"`, `rpy="0 0 0"`.
5. Glove message format, IMU model/frequency, deadman button, and any finger-bend sensors.
6. Independent physical E-stop and driver watchdog behavior.

Gyroscopes alone provide only angular velocity/orientation; they do not reliably measure the hand's absolute position in space or individual finger bends. See [GLOVE_INTERFACE.md](docs/GLOVE_INTERFACE.md) for details.

## Validation status

- Xacro opened in both `none` and `sim` modes.
- Generated URDF: 26 links, 25 joints, one `world` root, and 13 commandable joints.
- The URDF tree, mimic ratios, limits, mesh URIs/hashes, ros2_control, and SRDF passed static tests.
- `arm_o7_twin` pure safety tests: 18/18 passed.
- O7 adapter mapping/interpolation/mux pure-logic tests: 10/10 passed.
- Runtime validation on Ubuntu 24.04 / ROS 2 Jazzy / WSL2 built 5 packages; 28 tests passed with 0 errors and 0 failures.
- Synchronized Gazebo/RViz joint-state motion, four active controllers, and the combined arm-hand demo produced `COMBINED_SIM_TEST=PASS`.

## Sources and license

- LinkerHand O7: upstream commit
  `075cc7d42cc1e756bdcbece0fc069a0779fc5237`, Apache-2.0.
- ARM1.5 assets came from a user archive; the archive contained no license.
- Details: `src/arm_o7_description/SOURCE_ASSETS.md`, `LICENSES/`, and `ASSET_MANIFEST.sha256`.

Read [ARCHITECTURE.md](docs/ARCHITECTURE.md) for the technical flow and topic contract, and [HARDWARE_INTEGRATION.md](docs/HARDWARE_INTEGRATION.md) for physical commissioning.

For an experiment that sends simulation commands between two computers/Raspberry Pis, follow [NETWORK_TWO_HOSTS.md](docs/NETWORK_TWO_HOSTS.md). To publish the project to GitHub for the team, follow [GITHUB_PUBLISH.md](docs/GITHUB_PUBLISH.md).
