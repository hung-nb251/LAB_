# HC10DTP Planar Co-Drawing

Co-drawing is an independent pipeline. It does not launch or modify the camera
tracking path used by co-carrying.

## Data flow

```text
/joint_states -> ee_tracker -> trajectory_predictor -> x_d
/axia/human_force -------------------------------> F_h
                         x_d + planar admittance -> x_r
                         x_r -> Cartesian IK -> HC10DTP
/joint_states effort -> sensorless_force -> f_robot (logging only)
```

The controller implements

```text
M * e_ddot + D * e_dot + K * e = F_h
e = x_r - x_d
```

Only X and Y are controlled. Drawing-tip Z and end-effector orientation are
captured automatically at `Start Run` and remain fixed. The drawing tip is
configured 0.18 m from `tool0`; verify the sign of this offset on the real
robot before contact testing.

## Isolation from co-carrying

- Co-carrying continues to use `hrc_bringup/cocarry_real_gui.launch.py`.
- Co-drawing uses `codrawing_control/codrawing_real_gui.launch.py`.
- Co-drawing never launches a camera tracker or the camera coordinate transform.
- Co-drawing has its own YAML parameters and writes only to `codrawing_logs/`.
- Do not launch co-carrying and co-drawing simultaneously because both command
  `/cartesian_streamer/target_pose` and use the same physical robot.

## Build and start

On one computer, use three terminals:

```bash
# Terminal 1: robot / MotoROS2 communication
cd ~/cocarry_ws
./start_microros.sh

# Terminal 2: Axia EtherCAT/UDP driver
cd ~/cocarry_ws
./run_sensor_driver.sh

# Terminal 3: co-drawing pipeline
cd ~/cocarry_ws
source install/setup.bash
ros2 launch codrawing_control codrawing_real_gui.launch.py
```

Build after source changes:

```bash
cd ~/cocarry_ws
colcon build --symlink-install --packages-select \
  hc10dtp_bringup predictor_ui codrawing_control
source install/setup.bash
```

## Operating sequence

1. Move the robot to the drawing start pose at low teach-pendant speed.
2. Keep the robot stationary and the tool free of external contact.
3. In the Axia window, verify Yaw Z = -90 degrees, turn off `Calib Mode`,
   then press `Calibrate F/T Sensor`.
4. In the main UI, select SVGP or SVGP+MJM.
5. Press `Enable Robot`, then `Start Run`.
6. `Start Run` automatically captures the drawing tip, fixed Z, orientation,
   and workspace origin. Manual `Capture Init Pose` remains available but is
   not required in co-drawing mode.
7. Use `Stop Run` before leaving the drawing area.

Initial workspace relative to the captured drawing tip:

```text
X: -0.5 m .. +0.5 m
Y:  0.0 m .. +1.0 m (forward_sign_y = +1)
Z: fixed
```

Confirm the physical forward direction before contact testing. If robot
forward is `-Y`, set `forward_sign_y: -1.0`.

Also verify the force direction in free space. If pushing toward robot `+X` or
`+Y` produces the opposite motion, change `force_sign_x` or `force_sign_y` in
the co-drawing YAML; do not alter the camera/co-carry transform.

The Axia output uses one radial 4 N deadband. The controller does not apply a
second intent deadzone. Therefore a force at or below 4 N intentionally
produces no motion.

## Runtime safety interlocks

- `/cartesian_streamer/current_pose` is computed from live `/joint_states`,
  not copied from the requested command.
- Co-drawing cannot start until the streamer publishes `READY`, the Axia data
  is calibrated/live, and robot pose feedback is fresh.
- Target Z is locked within 5 mm and actual EE Z within 10 mm.
- Three consecutive IK failures, an IK branch jump, a safe joint-limit
  violation, stale joint feedback, queue rejection, or more than 50 mm of
  persistent Cartesian tracking error stops trajectory mode.
- MoveIt uses the installed KDL plugin. The previous TRAC-IK configuration was
  removed because `trac_ik_kinematics_plugin` is not installed on machine
  `hungnb`; that mismatch caused `No kinematics solver` messages.
- An XYZ workspace box is only a geometric guard. A point inside it can still
  be unreachable with fixed tool orientation or lie near a singularity; that
  case is handled as an IK fault, never by clamping individual joints.

Inspect status while testing:

```bash
ros2 topic echo /cartesian_streamer/status
ros2 topic echo /codrawing/status
```

## Logs

CSV files are written to `~/cocarry_ws/codrawing_logs`. They contain robot EE
and drawing-tip pose, nominal prediction, admittance correction, final position
reference, ATI human force, and sensorless robot force. Joint position,
velocity, and effort columns are intentionally omitted.

`f_robot_x/y/z` is an estimate from joint effort and the Jacobian. It is used
for analysis only, never as the admittance input. If `/joint_states.effort` is
empty, these fields will remain unavailable and the sensorless-force node will
warn.
