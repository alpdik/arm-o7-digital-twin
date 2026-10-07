# arm_o7_glove

Control the O7 hand with the 5DT Data Glove 5 Ultra. The glove drives the hand in
Gazebo and RViz, and the real Ti5 hand can mirror it.

```text
5DT glove ─► glove_5dt ─► /glove/finger_curls ─► teleop_panel ─► /twin/joint_trajectory
                                                     │    │               │
                           calibrate + ON/OFF toggle ┘    │      arbiter (SIM_ONLY)
                                                          │               ▼
                              /glove_teleop/hand_target ◄─┘   Gazebo + RViz
                                        │
                                        ▼
                                   ti5_bridge ─► RS485 ─► Ti5 hand
```

The real hand follows the glove directly. On this PC the simulated fingers move
at most ~0.3 rad/s of wall time (Gazebo runs at ~0.3x real time, and the model
limits fingers to 1 rad/s), so the sim lags during fast moves and then catches
up. Set the bridge's `source: sim` to make the real hand copy the measured
simulated hand instead, which is slower but never drifts from the sim. The
arbiter's `TWIN_COMMAND` mode is not used: it needs complete real arm and hand
states, and no real arm is connected.

## Run

Start terminal 1 (Gazebo) and terminal 2 (robot) as usual, then:

```bash
cd ~/arm_o7_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch arm_o7_glove glove_teleop.launch.py              # sim + real hand bridge
ros2 launch arm_o7_glove glove_teleop.launch.py real_hand:=false   # sim only
```

A window opens:

- **Calibrate glove** — required every time the panel starts (ON stays greyed
  out until it succeeds), and can be redone whenever it is OFF. Follow the
  prompts: open hand flat (3 s countdown, 2 s recording), then a tight fist with
  the thumb bent. A finger that barely changes fails the calibration.
- **OFF / ON** — while ON, the glove drives the hand. OFF (or closing the window)
  stops it, and the hand holds its position.
- **Also move the REAL Ti5 hand** — the real hand moves only while this is
  ticked *and* the panel is ON.

If the arbiter trips (shown under "arbiter"), click OFF and then ON to clear it.

## Safety

- The bridge sends nothing unless the panel's enable, the arbiter's `SIM_ONLY`
  mode and the hand target are all fresh (0.3 s). Otherwise the hand holds.
- The torque cap is 150/1000 by default (`config/glove_teleop.yaml`). Motion is
  eased at 6 angle units per frame (full travel in ~0.5 s), and angles are
  clamped to 0..90.
- This is an on/off toggle, not a held deadman: turn it OFF before taking the
  glove off.
- With `source: sim`, anything else that moves the simulated hand (for example
  a MoveIt demo) also moves the real hand while the panel is ON.

## Calibration and mapping

- The panel's calibration sets the glove node's `open_raw`/`closed_raw` at
  runtime. The values in `config/glove_teleop.yaml` are only a fallback. Finger
  order is `sensor_fingers` there (`ros2 topic echo /glove/finger_curls` shows
  curls in `position` and raw counts in `effort`).
- The single thumb sensor drives `thumb_cmc_pitch` (Ti5 thumb). Thumb roll and
  yaw (Ti5 thumb-base) stay open.
- O7 joint ranges map linearly onto the Ti5's 0..90 travel. This is provisional:
  check it on the real hand at low torque.
- `ti5_protocol.py` is a copy of `~/Desktop/ti5.py`.

## Devices

`udev/99-arm-o7-glove-hand.rules` gives the `plugdev` group access to the glove
(hidraw) and the hand's CH340 RS485 adapter, and creates `/dev/ti5_hand`.
Install it once per machine:

```bash
sudo cp ~/arm_o7_digital_twin/src/arm_o7_glove/udev/99-arm-o7-glove-hand.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

Your user must be in `plugdev` (Ubuntu desktop users are by default).
