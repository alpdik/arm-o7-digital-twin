# ARM1.5 + LinkerHand O7 Digital Twin Project

**Progress and technical concepts report — September 11, 2026**
## Project objective

The objective is to create a digital twin of the ARM1.5 robotic arm and LinkerHand O7 using Gazebo Harmonic, ROS 2 Jazzy, MoveIt 2, and RViz. In later phases, arm commands will come from gyroscope data and hand/finger commands from a sensor glove; motion will be executed bidirectionally and synchronously between the digital twin and the physical robot. The first physical experiment environment will be a plastic mannequin.
## Environment and sources

- Ubuntu 24.04 LTS, WSL2
- ROS 2 Jazzy
- Gazebo Harmonic
## How does the system work?

The digital twin is not one program. Several layers built on one another work together:
```text
Gyroscope / IMU ──> arm command mapper ──> MoveIt Servo ──> arm_controller

Glove ────────────> finger command mapper ─────────────────> hand_controller
                                                                  │
                                                                  v
                                    Gazebo simulation or physical robot
                                                                  │
                                                                  v
                                      joint_states feedback ──> RViz
Gazebo calculates how the robot behaves under physical rules. RViz displays the state known to ROS 2. MoveIt plans a path to the target, while MoveIt Servo provides real-time control from continuously changing inputs such as a gyroscope. Controllers apply the calculated joint commands to the simulation and later to real motor drivers.
### Difference between URDF and SRDF

**URDF (Unified Robot Description Format)** is the robot's physical and kinematic description. It contains links, joints, joint axes, motion limits, visual and collision meshes, masses, and inertial values. Its everyday equivalent is a combination of a person's skeleton and a technical drawing: it says which bones connect and how far each joint can rotate.
**SRDF (Semantic Robot Description Format)** describes how MoveIt should use the body created by the URDF. It contains planning groups, named poses, end-effector information, passive joints, and safe collision pairs to ignore during planning. Its everyday equivalent is a user manual and saved seat settings rather than the vehicle's technical drawing. `ready` and `four_finger_fist` are saved settings; MoveIt recalculates how to reach them from the current state.
Therefore, an incorrect joint axis in the URDF builds the robot's body incorrectly, while an incorrect planning group in the SRDF makes MoveIt use the correctly built body with the wrong joints.
## Core terms and plain-language equivalents

| Term | Technical meaning | Everyday analogy |
|---|---|---|
| Digital twin | A numerical model representing the state and behavior of a real system | The aircraft in a flight simulator continuously representing the real aircraft |
| ROS 2 | Infrastructure for robot software communication and coordination | The body's nervous system and a city's postal network |
| Node | A ROS 2 program that performs one task | A specialist worker in a factory |
| Topic | A continuous data stream from one source to one or more listeners | A radio channel that broadcasts continuously |
| Service | A short operation that returns one response to a request | Asking a question at an information desk |
| Action | A long-running, observable, cancellable task | Placing a restaurant order and tracking its preparation |
| URDF | A file describing the robot's links, joints, geometry, and physical properties | A skeleton and manufacturing drawing |
| Xacro | A template language that generates repeated URDF parts with parameters | A mold that produces products in different sizes |
| SRDF | A file describing MoveIt planning groups, named poses, and semantic information | A user manual and saved seat positions |
| Link | A rigid part of the robot | An arm bone or finger segment |
| Joint | A movable connection between two links | An elbow or door hinge |
| Mesh | A three-dimensional surface model of a robot part | An object's outer shell or skin |
| Collision geometry | Geometry used for collision calculations | A protective volume around an object |
| TF / frame | The coordinate relationship between parts | A position and orientation description on a map |
| Planning group | A set of joints planned together by MoveIt | A muscle group working on one motion |
| Named state | A target pose saved as named joint values | A memory-seat button in a car |
| Trajectory | A motion path containing joint targets and arrival times | A route with stops and durations |
| Controller | Software that applies target joint values to simulation or motors | A mechanism that carries a brain's command to the muscles and regulates motion |
| ros2_control | The standard layer between controllers and simulation or hardware | A standard transmission/interface that fits different motors |
| MoveIt 2 | A kinematics, collision-checking, and motion-planning system | A navigation app that plans around obstacles |
| MoveIt Servo | A system that turns continuous velocity, joint, or end-effector commands into live motion | Driving with a steering wheel instead of following a prepared route |
| Gazebo | A robot simulator that calculates physics, gravity, and contact | A virtual laboratory for robots |
| RViz | An interface that visualizes ROS 2 data and robot state | A vehicle dashboard, not the engine itself |
| Mimic joint | A joint that automatically follows another joint at a fixed ratio | A gear-driven mechanism attached to a main shaft |
| Joint state | Feedback about a joint's position, velocity, or effort | A car speedometer and sensor panel |
| Namespace | A prefix that separates similar names under different systems | Two people with the same name registered in different departments |
| rosbag | A recording system that saves and replays ROS 2 messages with timestamps | A robot's black box |
## Relationship between MoveIt, RViz, and named motions

Items defined in the SRDF as `home`, `ready`, `open`, `pregrasp`, or `four_finger_fist` are target poses, not fully pre-recorded motions. When **Plan & Execute** is selected in RViz, MoveIt reads the current joint state, evaluates collisions and limits, and generates a new trajectory for that moment.
Test commands sent directly to the `/sim/.../follow_joint_trajectory` action bypass the MoveIt planner and go to the controller. They were used to test controller tracking and Gazebo–RViz synchronization.

Named poses remain useful after the gyroscope and glove are added. They can serve as calibration starts, teleoperation-ready states, automatic gestures, recovery after stopping, and end-of-task parking poses. Continuous user motion will instead use current commands generated from sensors.
An example hybrid task flow is:

```text
IDLE / waiting
   ↓
CALIBRATION / gyroscope and glove calibration
   ↓
READY / move to ready pose with MoveIt
   ↓
TELEOP / continuous control with gyroscope and glove
   ├── run a named gesture
   ├── move to a specific arm pose
   ├── hold or stop motion
   └── return to user control
   ↓
OPEN + HOME / finish the task and park
## Completed work

1. The ARM1.5 arm and LinkerHand O7 right-hand models were combined in one robot description. The arm-hand mounting transform is parameterized.
2. A workspace of five ROS 2 packages was created. All 28 tests passed without errors in the latest validation.
3. The Gazebo world, robot spawning, `gz_ros2_control`, and ROS-Gazebo clock bridge were prepared.
4. Arm, hand, and joint-state controllers were enabled. `FollowJointTrajectory` action servers for the arm and hand were verified.
5. The Gazebo and RViz robot models became visible; joint-state flow synchronized motion in both interfaces.
6. MoveIt 2 planning groups `arm`, `hand`, and `arm_with_hand` were defined. Arm states `home` and `ready`, and hand states `open`, `pregrasp`, `four_finger_fist`, and `contact_closed` were created.
7. Arm `home`–`ready` motion and direct trajectory action commands were successfully tested in Gazebo and RViz. A 60% speed scale was suitable for RViz tests.
8. Self-collision at the hand's fully closed target was investigated. State-validity scanning found 64% closed valid and 65% closed invalid; `pregrasp` at 62% was selected as the safe approach state.
9. The `open` state was defined with a 0.001 rad buffer so tiny numerical deviations at a zero joint limit do not cause a MoveIt start-state error.
10. PIP/DIP fingertip mimic ratios were measured. All ten follower joints, including the thumb, matched the expected URDF ratios with zero measurement error.
11. The `four_finger_fist` motion, with the other four fingers closed while the thumb is open, passed validity, controller, and mimic tests. With suitable arm/wrist orientation, this forms the hand portion of a thumbs-up motion.
12. The previously temporary path-tolerance error on `thumb_cmc_roll` was investigated again. All targets in the 0.001–0.279 rad range completed with zero tracking error; no persistent controller or model fault was observed in this range.
13. In the combined test where the arm and hand move simultaneously, both outbound and return motions completed successfully: `COMBINED_SIM_TEST=PASS`.
14. Simulation and future real-robot paths were separated with `/sim` and `/real` namespaces. Current test scripts command only `/sim` actions and send no data to the physical robot.
15. The control architecture was clarified so gyroscope data controls the arm and glove data controls the hand and fingers. The proposed inputs are `/gyro/imu` and `/glove/finger_curls`.
## Current status

The digital twin's core robot model, ROS 2 controller layer, MoveIt planning infrastructure, and Gazebo–RViz motion synchronization are working. Arm and four-finger-fist motions have been verified separately and simultaneously. No motion test has yet been performed with the physical robot.
## Next work

### 1. Complete the motion and gesture library

Retest `pregrasp` including thumb opposition, then add thumbs-up, pointing, pinch, and other grasp forms. For every target, run MoveIt validity, self-collision, controller tracking, and Gazebo–RViz matching tests. This will validate the model's reachable motion space before the glove is connected.
### 2. Establish continuous arm control

Develop a `gyro_mapper` ROS 2 node that receives gyroscope/IMU messages. Filter noise and sudden jumps, then convert measurements into end-effector orientation, velocity, or joint-jog commands accepted by MoveIt Servo. MoveIt Servo will perform like driving with a steering wheel instead of playing a prepared route.

A gyroscope alone does not provide reliable absolute three-dimensional position. A full 6-DOF hand pose may require a button, position tracker, camera, or another measurement source in addition to orientation. The first experiment will therefore define explicitly which arm motions correspond to each gyro axis.
### 3. Build the glove–O7 mapping

Develop a `glove_mapper` node to read the glove's finger sensors. Determine open and closed calibration values for each sensor, normalize them to 0–1, and convert them into the O7's seven active joint targets. PIP/DIP follower joints will track automatically using the verified mimic ratios. Filtering and short trajectory generation will keep motion smooth and continuous.
### 4. Develop the hybrid task manager

Create a ROS 2 state-machine/action node that transitions between named MoveIt poses and live teleoperation. The first version will contain `IDLE`, `CALIBRATION`, `READY`, `TELEOP`, `HOLD`, `RECOVERY`, and `HOME`. In more complex mannequin applications, approach, user control, contact, waiting, and retract steps can be combined with MoveIt Task Constructor or a similar task planner.
### 5. Add measurement and experiment recording

Record sensor input, generated targets, Gazebo/robot feedback, and system state in a rosbag with shared timestamps. Automatically report command frequency, end-to-end latency, jitter, packet loss, and joint tracking error. These records will support comparison with the networking team and future paper preparation.
### 6. Complete `/sim` and `/real` integration

First validate every command in Gazebo through the `/sim` path. When the networking and physical-robot teams are ready, connect the same high-level command contract to the `/real` backend. For the digital twin to be a true mirror, not only sent commands but also joint states returned by the real robot must be forwarded to Gazebo/RViz.
## Main stored test records

- `build_four_finger_fist.log`
- `combined_motion_fist.log`
- `four_finger_mimic_check.log`
- `thumb_roll_range.log`
- `terminal1_gazebo.log`
- `terminal2_robot.log`
