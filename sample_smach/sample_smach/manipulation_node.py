from ai_robot_book_interfaces.srv import StringCommand

import rclpy
from rclpy.node import Node

import random
from time import sleep

class ManipulationServer(Node):

    def __init__(self):
        super().__init__('manipulation_server')
        self.srv = self.create_service(
            StringCommand, 'manipulation/command', self.command_callback)

    def command_callback(self, request, response):
        self.get_logger().info(f"{request.command}")

        sleep(1)
        
        prob = random.random()
        self.get_logger().info(f"Try to manipulate the target object")

        if 0.7 > prob:
            self.get_logger().info("Succeeded to manipulate the target object")
            response.answer = "succeeded"
        else:
            self.get_logger().info("Failed to manipulate the target object")
            response.answer = "failed"

        return response


def main(args=None):
    rclpy.init(args=args)

    manipulation_server = ManipulationServer()

    rclpy.spin(manipulation_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
