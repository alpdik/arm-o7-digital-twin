# Digital twin architecture

## Namespace and data ownership

| Interface | Simulation | Physical |
|---|---|---|
| Controller manager | `/sim/controller_manager` | `/real/controller_manager` |
| Kol controller | `/sim/arm_controller` | `/real/arm_controller` |
| El controller | `/sim/hand_controller` | `/real/hand_controller` |
| State | `/sim/joint_states` | `/real/joint_states` |

State ownership on the physical side is:

```text
arm driver -> /real/arm_joint_states  \
                                         joint_state_mux -> /real/joint_states
O7 adapter  -> /real/hand_joint_states /
```

The mux must be the only publisher of the combined `/real/joint_states` topic. The arbiter's SHADOW and tracking-error logic expects this complete, fresh message containing six arm joints and seven independent hand joints.

The canonical command input is `/twin/joint_trajectory`. It must have only one command source: the glove adapter, operator UI, or an approved plan executor. Do not use MoveIt's direct `/sim` action execution and arbiter topic execution at the same time.

## Modes

| Mode | Sim command | Real command | Purpose |
|---|---:|---:|---|
| `SAFE_IDLE` | no | no | Startup, fault, and safe idle |
| `SIM_ONLY` | yes | no | Bring up the model/controller |
| `SHADOW` | from real state | never | Show physical motion in Gazebo |
| `TWIN_COMMAND` | yes | yes | Split the same safe reference to both sides |
| `REAL_ONLY` | no | yes | Controlled physical commissioning |

Direct transitions between active modes are rejected. `SAFE_IDLE` is required first. Releasing the deadman, a heartbeat/state timeout, or a limit/tracking error blocks new commands and latches the fault.

## QoS and time

- Command, heartbeat, deadman ve mode: reliable.
- Joint state: sensor-data profile; low latency, limited queue.
- Mode/status: transient-local, so the last state reaches new subscribers.
- Sim nodes use `use_sim_time=true`; physical drivers use `false`.
- `/clock` is bridged only from Gazebo to ROS.
- If using different computers, keep wall clocks close with NTP/chrony; do not copy Gazebo timestamps into physical execution.

## Networking

Two Ubuntu terminals on one computer require no extra DDS configuration. On separate computers:

1. Use the same reliable wired LAN and the same `ROS_DOMAIN_ID`.
2. Keep sim and real topics in namespaces.
3. If multicast is blocked, configure static peer/unicast settings for the selected RMW.
4. Separate the command network from the corporate/public network; open only the required DDS traffic in the firewall.
5. On network loss, the driver's own watchdog must perform a controlled stop; a ROS node is not a physical E-stop.

## Why not connect joint state directly to command?

Bidirectional topic copying creates the risk of delayed positive feedback, oscillation, and two controllers claiming the same interface. `TWIN_COMMAND` sends the same reference to both sides; `sim_state` and `real_state` are only observed and used for error calculation. `SHADOW` is one-way and explicitly real→sim.
