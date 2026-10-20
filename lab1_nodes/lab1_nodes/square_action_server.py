import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from lab1_interfaces.action import GoToPose, DrawSquare

START_X = 5.544445
START_Y = 5.544445
MAX_SIDE = 5.0   # keeps the square inside the 0..11 window


class SquareActionServer(Node):
    def __init__(self):
        super().__init__('square_action_server')
        self.group = ReentrantCallbackGroup()

        self.reset_cli = self.create_client(
            Trigger, 'reset_turtle', callback_group=self.group)
        self.goto_cli = ActionClient(
            self, GoToPose, 'go_to_pose', callback_group=self.group)

        self.server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self.group)
        self.get_logger().info('draw_square server ready')

    def on_goal(self, goal_request):
        side = goal_request.side_length
        if side <= 0.0 or side > MAX_SIDE:
            self.get_logger().warn(
                f'side_length must be in (0, {MAX_SIDE}], rejected')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def wait(self, future):
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def fail(self, goal_handle, msg):
        self.get_logger().error(msg)
        goal_handle.abort()
        result = DrawSquare.Result()
        result.success = False
        return result

    def execute(self, goal_handle):
        side = goal_handle.request.side_length
        feedback = DrawSquare.Feedback()

        if not self.reset_cli.wait_for_service(timeout_sec=2.0):
            return self.fail(goal_handle, 'reset_turtle service not available')
        if not self.goto_cli.wait_for_server(timeout_sec=2.0):
            return self.fail(goal_handle, 'go_to_pose server not available')

        # 1) clean screen + known start pose
        reset = self.wait(self.reset_cli.call_async(Trigger.Request()))
        if not reset.success:
            return self.fail(goal_handle, 'reset failed')

        # 2) the 4 corners, counter-clockwise, each one a separate goal
        corners = [
            (START_X + side, START_Y,        math.pi / 2),
            (START_X + side, START_Y + side, math.pi),
            (START_X,        START_Y + side, -math.pi / 2),
            (START_X,        START_Y,        0.0),
        ]

        for i, (x, y, yaw) in enumerate(corners, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = DrawSquare.Result()
                result.success = False
                return result

            feedback.current_side = i
            goal_handle.publish_feedback(feedback)

            goal = GoToPose.Goal()
            goal.target.x = float(x)
            goal.target.y = float(y)
            goal.target.yaw = float(yaw)

            sent = self.wait(self.goto_cli.send_goal_async(goal))
            if not sent.accepted:
                return self.fail(goal_handle, f'side {i} goal rejected')

            # 3) wait for this goal to finish before sending the next
            res = self.wait(sent.get_result_async())
            if not res.result.success:
                return self.fail(goal_handle, f'side {i} failed')

        goal_handle.succeed()
        result = DrawSquare.Result()
        result.success = True
        return result


def main():
    rclpy.init()
    node = SquareActionServer()
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
