import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty, Trigger
from turtlesim_msgs.srv import SetPen, TeleportAbsolute

START_X = 5.544445
START_Y = 5.544445
START_THETA = 0.0


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.group = ReentrantCallbackGroup()

        # turtlesim's own services
        self.clear_cli = self.create_client(
            Empty, '/clear', callback_group=self.group)
        self.pen_cli = self.create_client(
            SetPen, '/turtle1/set_pen', callback_group=self.group)
        self.tp_cli = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute',
            callback_group=self.group)

        # our service
        self.create_service(
            Trigger, 'reset_turtle', self.on_reset,
            callback_group=self.group)
        self.get_logger().info('reset_turtle service ready')

    def call(self, client, request):
        future = client.call_async(request)
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def set_pen(self, off):
        req = SetPen.Request()
        req.r, req.g, req.b = 179, 184, 255   # turtlesim default color
        req.width = 3
        req.off = 1 if off else 0
        return self.call(self.pen_cli, req)

    def on_reset(self, request, response):
        for cli in (self.clear_cli, self.pen_cli, self.tp_cli):
            if not cli.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = 'turtlesim services not available'
                return response

        self.set_pen(off=True)                 # no line while teleporting
        tp = TeleportAbsolute.Request()
        tp.x, tp.y, tp.theta = START_X, START_Y, START_THETA
        self.call(self.tp_cli, tp)
        self.call(self.clear_cli, Empty.Request())
        self.set_pen(off=False)                # pen back on

        response.success = True
        response.message = 'Turtle reset to start'
        return response


def main():
    rclpy.init()
    node = ResetService()
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
