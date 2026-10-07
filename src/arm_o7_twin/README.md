# arm_o7_twin

Fail-closed ROS 2 Jazzy command arbiter for the ARM1.5 arm plus the seven
actuated joints of a Linker Hand O7. This package does not contain a hardware
driver. It sits between a canonical command producer (MoveIt, glove bridge, or
operator UI) and separately namespaced simulation/real controllers.

## Safety contract

The process always starts in `SAFE_IDLE`, and `enable_real_output` defaults to
`false`. A command is forwarded only when its mode permits the target, the
deadman is true, heartbeat and required joint states are fresh, all trajectory
fields are well formed and finite, positions are within configured limits, and
the first point is near the current state. `SHADOW` and `TWIN_COMMAND` also
interlock on sim-real tracking error. In `SHADOW`, complete real joint-state
samples are position/limit checked and converted at no more than 20 Hz to short
simulation-only trajectories. Canonical command messages are ignored in this
mode, so the data flow is unambiguously real-to-sim. A safety failure while
active latches a fault and returns to `SAFE_IDLE`.

Modes:

| Mode | Simulation | Real robot | Extra behavior |
|---|---:|---:|---|
| `SAFE_IDLE` | no | no | startup/fault state |
| `SIM_ONLY` | yes | no | simulation commissioning |
| `SHADOW` | yes | never | rate-limited real-state to sim mirroring |
| `TWIN_COMMAND` | yes | yes | identical accepted command to both |
| `REAL_ONLY` | no | yes | real-side state/start checks |

Real hardware must independently provide an e-stop, controller watchdog,
velocity/effort limits, and collision protection. A ROS topic bridge is not a
safety-rated device. An arbiter trip stops new forwarding; a topic-only bridge
cannot guarantee cancellation of a trajectory already accepted by a
controller. The YAML contains the configured model limits; verify them against
the physical hardware/manufacturer limits before hardware use.

## Interfaces

Inputs:

- `/twin/joint_trajectory` (`trajectory_msgs/JointTrajectory`), containing a
  complete arm group, complete hand group, or both by default
- `/twin/deadman` (`std_msgs/Bool`), continuously published and asserted by the
  operator; its sample also expires after 0.30 s
- `/twin/heartbeat` (`std_msgs/Empty`), expected faster than the configured
  0.30 s timeout
- `/twin/mode_command` (`std_msgs/String`), one of the five mode names
- `/sim/joint_states` and `/real/joint_states` (`sensor_msgs/JointState`)

Outputs are split into arm and hand messages:

- `/sim/arm_controller/joint_trajectory`
- `/sim/hand_controller/joint_trajectory`
- `/real/arm_controller/joint_trajectory` (disabled by default)
- `/real/hand_controller/joint_trajectory` (disabled by default)
- `/twin/mode`, `/twin/status` (JSON), and `/diagnostics`

Each mode also has a `std_srvs/Trigger` service, for example
`/arm_o7_twin_arbiter/mode/sim_only`; fault reset is
`/arm_o7_twin_arbiter/clear_fault`. Active-to-active changes are rejected: call
the `safe_idle` service first. Clear a latched fault only in `SAFE_IDLE` with
the deadman released.

Canonical arm joints are `arm_joint_1` through `arm_joint_6`. Hand joints are
`thumb_cmc_roll`, `thumb_cmc_yaw`, `thumb_cmc_pitch`, `index_mcp_pitch`,
`middle_mcp_pitch`, `ring_mcp_pitch`, and `pinky_mcp_pitch`.

## Build and run (Ubuntu 24.04 / ROS 2 Jazzy)

```bash
cd ~/arm_o7_digital_twin
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm_o7_twin
source install/setup.bash
ros2 launch arm_o7_twin arm_o7_twin.launch.py
```

Example simulation arming sequence (after `/sim/joint_states` is available):

```bash
ros2 topic pub -r 10 /twin/heartbeat std_msgs/msg/Empty '{}'
ros2 topic pub -r 10 /twin/deadman std_msgs/msg/Bool '{data: true}'
ros2 service call /arm_o7_twin_arbiter/mode/sim_only std_srvs/srv/Trigger '{}'
```

Run the ROS-independent safety tests from this package directory:

```bash
python3 -m unittest discover -s test -v
```

For a ROS build, also run `colcon test --packages-select arm_o7_twin` and inspect
the results with `colcon test-result --verbose`.

## Local digital-twin qualification

With Gazebo, controllers, MoveIt, and RViz already running, execute:

```bash
cd ~/arm_o7_digital_twin
bash scripts/run_qualification.sh 1
```

The qualification executable reads named states from the installed SRDF, checks
the controller lifecycle states and MoveIt state validity, measures joint-state
rate and Gazebo real-time factor, then plans and executes a repeatable motion
sequence through `/sim/move_action`.  It writes machine-readable JSON and CSV
evidence.  `contact_closed` is validity-diagnostic only and is never executed as
a collision-free planning goal.
