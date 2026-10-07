# Gyroscope arm and glove O7 control interface

## Input separation

- The separate gyroscope / IMU unit is the source of arm motion commands.
- The glove is the source of O7 hand and finger motion commands.
- These streams are processed separately with timestamps and sent to the same simulation or physical backend only after the safety arbiter.

## Measurable and not measurable

An IMU/gyroscope directly provides angular velocity and usually orientation through a filter. On its own, it cannot:

- Measure the hand's absolute XYZ position stably for a long time; integrating acceleration drifts quickly.
- Measure the separate bend angles of the four fingers and thumb.

The recommended arm-side sensor set is therefore:

- Wrist orientation: calibrated IMU quaternion.
- Wrist XYZ: optical tracking, UWB, camera/AprilTag, or another external reference.
- Safety: a physical hold-to-run deadman button.

On the O7 side, the glove must provide finger bends through flex/Hall/encoder sensors or the glove manufacturer's joint estimates.

## Recommended ROS contract

```text
/gyro/imu                  sensor_msgs/Imu
/glove/finger_curls        sensor_msgs/JointState
/twin/deadman              std_msgs/Bool
/twin/heartbeat            std_msgs/Empty
```

For the arm, filtered IMU data from the gyroscope can be converted to `geometry_msgs/PoseStamped` or `geometry_msgs/TwistStamped` and sent to MoveIt Servo. After calibration curves are applied, finger sensor values are converted to canonical O7 joint angles and added to the `/twin/joint_trajectory` command.

## Calibration sequence

1. Measure gyro bias and quaternion norm while the glove is stationary.
2. Record the `glove_frame -> robot_base` transform with the user in a neutral pose.
3. Initially limit the motion scale to 10–20% in simulation.
4. Enable workspace clamping, singularity slowdown, and collision checking.
5. Test that releasing the deadman stops the new command stream in less than 300 ms.
6. Test packet loss, sensor freeze, NaN values, quaternion norm errors, and timeouts.

Adding a random IMU→joint mapping without knowing the glove message format and sensor model is unsafe. Complete this adapter in the second phase using a real message sample (`ros2 topic echo --once`) and calibration data.
