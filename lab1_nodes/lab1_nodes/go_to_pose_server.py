import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose

from lab1_interfaces.action import GoToPose
from lab1_interfaces.srv import SetSpeedLimits

POS_TOL = 0.1    # meters
YAW_TOL = 0.05   # radians
K_LIN = 1.5
K_ANG = 4.0
DT = 0.02        # control loop period (s)


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')
        self.group = ReentrantCallbackGroup()

        # R7: speed limit parameters
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.add_on_set_parameters_callback(self.on_params)

        self.pose = None
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(
            Pose, '/turtle1/pose', self.on_pose, 10,
            callback_group=self.group)

        # service that changes the parameters while running
        self.create_service(
            SetSpeedLimits, 'set_speed_limits', self.on_set_limits,
            callback_group=self.group)

        # R3: action server
        self.action = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self.group)

        self.get_logger().info('go_to_pose server ready')

    # ---------- parameters ----------
    def on_params(self, params):
        for p in params:
            if p.name in ('max_linear_speed', 'max_angular_speed') and p.value <= 0.0:
                return SetParametersResult(
                    successful=False, reason='speed must be > 0')
        return SetParametersResult(successful=True)

    def on_set_limits(self, request, response):
        if request.max_linear <= 0.0 or request.max_angular <= 0.0:
            response.success = False
            response.message = 'Both speeds must be greater than 0'
            return response
        self.set_parameters([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE,
                      float(request.max_linear)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE,
                      float(request.max_angular)),
        ])
        response.success = True
        response.message = 'Speed limits updated'
        return response

    # ---------- pose ----------
    def on_pose(self, msg):
        self.pose = msg

    # ---------- action ----------
    def on_goal(self, goal_request):
        t = goal_request.target
        if not (0.0 <= t.x <= 11.0 and 0.0 <= t.y <= 11.0):
            self.get_logger().warn('Goal outside window (0..11), rejected')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def limits(self):
        return (self.get_parameter('max_linear_speed').value,
                self.get_parameter('max_angular_speed').value)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def finish(self, goal_handle, ok, dist):
        self.stop()
        result = GoToPose.Result()
        result.success = ok
        result.final_distance = float(dist)
        if ok:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def execute(self, goal_handle):
        t = goal_handle.request.target
        feedback = GoToPose.Feedback()
        start = time.time()

        while self.pose is None and rclpy.ok():
            time.sleep(0.05)

        # phase 1: go to (x, y)
        while rclpy.ok():
            if goal_handle.is_cancel_requested or time.time() - start > 60.0:
                return self.finish(goal_handle, False, 0.0)

            dx = t.x - self.pose.x
            dy = t.y - self.pose.y
            dist = math.hypot(dx, dy)
            if dist < POS_TOL:
                break

            angle_err = normalize(math.atan2(dy, dx) - self.pose.theta)
            max_lin, max_ang = self.limits()

            cmd = Twist()
            if dist > 0.25:
                cmd.angular.z = clamp(K_ANG * angle_err, max_ang)
            if abs(angle_err) < 0.5:
                cmd.linear.x = clamp(K_LIN * dist, max_lin)
            self.cmd_pub.publish(cmd)

            feedback.remaining_distance = float(dist)
            goal_handle.publish_feedback(feedback)
            time.sleep(DT)

        self.stop()

        # phase 2: rotate to final yaw
        while rclpy.ok():
            if goal_handle.is_cancel_requested or time.time() - start > 60.0:
                return self.finish(goal_handle, False, 0.0)

            yaw_err = normalize(t.yaw - self.pose.theta)
            if abs(yaw_err) < YAW_TOL:
                break
            _, max_ang = self.limits()
            cmd = Twist()
            cmd.angular.z = clamp(K_ANG * yaw_err, max_ang)
            self.cmd_pub.publish(cmd)

            feedback.remaining_distance = float(
                math.hypot(t.x - self.pose.x, t.y - self.pose.y))
            goal_handle.publish_feedback(feedback)
            time.sleep(DT)

        final = math.hypot(t.x - self.pose.x, t.y - self.pose.y)
        return self.finish(goal_handle, True, final)


def main():
    rclpy.init()
    node = GoToPoseServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
