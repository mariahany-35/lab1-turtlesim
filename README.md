# Lab 1: ROS 2 and Turtlesim (Design of Mechatronics Systems)

Alexandria University, Faculty of Engineering. ROS 2 Lyrical Luth on Ubuntu 26.04.

## Team
| Name | ID |
|------|----|
| Maria Hany | 9762 |
| Marina Moheb | 9758 |
| Zeina Ahmed | 9553 |

## Packages
- **lab1_interfaces** (ament_cmake): `msg/Target` (x, y, yaw), `action/GoToPose` (goal = Target; feedback = remaining distance), `action/DrawSquare` (goal = side length; feedback = current side), `srv/SetSpeedLimits` (max_linear, max_angular).
- **lab1_nodes** (ament_python):
  - `go_to_pose_server`: action server `/go_to_pose`. Drives the turtle to (x, y), then rotates to yaw. Owns the parameters `max_linear_speed` and `max_angular_speed` (read on every control step, so changes apply live) and the service `/set_speed_limits`.
  - `reset_service`: service `/reset_turtle`. Uses turtlesim's own `/clear`, `/turtle1/set_pen` and `/turtle1/teleport_absolute` (pen off while teleporting).
  - `square_action_server`: action server `/draw_square`. Calls reset, then sends 4 separate goals to `/go_to_pose`, waiting for each to finish.
- **lab1_bringup** (ament_python): `launch/lab1.launch.py` and `config/params.yaml`.

## Build
```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone <this-repo-url> .
cd ~/ws
colcon build
source install/setup.bash
```

## Run everything (one command)
```bash
ros2 launch lab1_bringup lab1.launch.py
```
Speed limits come from `lab1_bringup/config/params.yaml`. Rebuild after editing it.

## Try each requirement (in a second terminal, after sourcing the workspace)
```bash
# R1: turtlesim and nodes are up
ros2 node list

# R3: goToPose action with feedback
ros2 action send_goal /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 8.0, y: 8.0, yaw: 1.57}}" --feedback

# R4: reset service
ros2 service call /reset_turtle std_srvs/srv/Trigger

# R5: square action (4 goToPose calls)
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 3.0}" --feedback

# R7: parameters
ros2 param list /go_to_pose_server
ros2 param get /go_to_pose_server max_linear_speed
ros2 param set /go_to_pose_server max_linear_speed 1.0

# R7: change limits live with our service (invalid values are refused)
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear: 0.5, max_angular: 1.0}"
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear: -1.0, max_angular: 1.0}"
```

## Notes
- In Lyrical the turtlesim messages and services live in the `turtlesim_msgs` package.
- Goals outside the window (0 to 11) are rejected; `side_length` must be in (0, 5].
