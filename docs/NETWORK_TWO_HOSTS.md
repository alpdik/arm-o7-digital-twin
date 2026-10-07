# ROS 2 network experiment with two computers / Raspberry Pi

## Purpose

The first network experiment has two roles:

```text
Sender device                                Simulation device
Raspberry Pi A                               Raspberry Pi B veya güçlü Ubuntu PC

network_sender_demo.sh                       Gazebo + ros2_control + MoveIt/RViz
        |                                                |
      +--- ROS 2 action / DDS network ----------------->+
                                                         |
                                             ARM1.5 + O7 hareketi
```

The sender device sends joint targets. ROS 2 controllers on the simulation device accept the target, move the Gazebo robot, and RViz shows the same motion through `/sim/joint_states`.

This first test uses only `/sim` actions. `/real`, which is the physical robot output, is not used. This is also a direct ROS 2/DDS test; DDS default transport is usually UDP/RTPS-based. If the research specifically needs to compare TCP protocols, add a TCP gateway/bridge separately in a later phase.

## Hardware note

ROS 2 Jazzy supports both 64-bit x86 and 64-bit ARM on Ubuntu 24.04. Gazebo Harmonic's official primary target is Ubuntu amd64; ARM architectures are best-effort. Therefore, Gazebo installation or GUI performance on a Pi is not guaranteed. The first reliable arrangement is:

- Raspberry Pi A: command sender only
- x86_64 Ubuntu bilgisayar: Gazebo, controller, MoveIt ve RViz

Simulation can be tried on a more powerful device such as a Pi 5; first collect the output of `scripts/preflight_report.sh`.

If Ubuntu in VirtualBox is the simulation device, select `Bridged Adapter` instead of `NAT` for the network adapter. Otherwise, the physical Raspberry Pi may not discover ROS 2 participants in the virtual machine.

## Shared network settings

Both devices must connect to the same local network and use the same ROS 2 Jazzy distribution. Run the first test over Ethernet if possible. In every new terminal on both devices:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/arm_o7_digital_twin
source scripts/network_env.sh
```

The script sets these shared values:

- `ROS_DOMAIN_ID=42`: groups ROS 2 participants in the same experiment.
- `ROS_LOCALHOST_ONLY=0`: does not restrict communication to the local computer.
- `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`: discovers devices on the same subnet.
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`: selects the same DDS implementation on both sides.

If multicast discovery is blocked, provide the other device's IP manually:

```bash
export ARM_O7_PEER_IP=192.168.1.52
source scripts/network_env.sh
```

Replace the IP address with the actual device address.

## 1. Discovery test

On the simulation device:

```bash
ros2 multicast receive
```

On the sender device:

```bash
ros2 multicast send
```

If the receiver sees the message, the multicast path works. Then start Terminal 1 and Terminal 2 normally on the simulation device after sourcing the shared network environment.

## 2. Remote action visibility

Gönderici cihazda:

```bash
ros2 action list -t | grep follow_joint_trajectory
```

Beklenen:

```text
/sim/arm_controller/follow_joint_trajectory
/sim/hand_controller/follow_joint_trajectory
```

If this list does not appear, do not send motion commands yet. Compare the domain, RMW, subnet, virtual-machine network mode, and firewall settings on both sides.

## 3. Verified simulation demo over the network

The sender Raspberry Pi does not need to build the full workspace. ROS 2 Jazzy, `control_msgs`, and the two network scripts in the repository are sufficient:

```bash
cd ~/arm_o7_digital_twin
source /opt/ros/jazzy/setup.bash
source scripts/network_env.sh
bash scripts/network_sender_demo.sh 2>&1 | tee network_sender_demo.log
```

Expected result:

```text
[outbound] ARM=SUCCEEDED HAND=SUCCEEDED
[return] ARM=SUCCEEDED HAND=SUCCEEDED
NETWORK_SIM_DEMO=PASS
```

The arm and hand should be seen moving out and back together in Gazebo and RViz. `elapsed_ms` is the approximate total time measured on the sender from action submission until both targets finish; it is not pure network latency.

## Next networking phase

After the first test proves DDS connectivity, add these layers with the networking team:

1. Sequence numbers and source timestamps on command and joint-state messages
2. One-way, round-trip, and jitter measurements
3. Packet-loss, latency, and bandwidth experiments
4. A sender node that converts glove/IMU data into joint targets
5. A safety-arbitrated path through `/twin/heartbeat`, `/twin/deadman`, and `/twin/joint_trajectory`
6. A TCP gateway in addition to the DDS/UDP baseline if required by the research

Do not enable physical `/real` output before verifying the driver protocol and limits.
