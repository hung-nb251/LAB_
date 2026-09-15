# industrial_msgs dependency

Added 2026-09-07 to resolve missing `industrial_msgs/msg/RobotStatus` locally.

- Upstream: https://github.com/ros-industrial/industrial_core.git
- Branch: `ros2_msgs_only`
- Commit: `d547cdcfdaf3bc0d46325215b8219b0a190c8e6c`
- Local clone: `src/industrial_core_ros2_msgs_only`
- Package version: 0.7.3

This is a separate Git clone, not a registered submodule. Preserve it separately
when transferring this workspace, or clone the upstream branch and check out
the pinned commit above. Do not source the Downloads workspace: its installed
package contains stale symlinks to another user's build tree.

Build after sourcing ROS Humble:

```bash
colcon build --symlink-install --packages-select industrial_msgs
source install/setup.bash
ros2 interface show industrial_msgs/msg/RobotStatus
```

No robot configuration, force scale, Servo state or motion commands were changed.
Local message serialization checks are not a substitute for receiving a real
controller message after sourcing the updated workspace.
