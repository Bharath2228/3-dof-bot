#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from arduinobot_msgs.action import ArduinobotTask


class CommandInterpreter(Node):
    def __init__(self):
        super().__init__('command_interpreter')

        self.latest_positions = {}

        self.create_subscription(PointStamped, '/detected_objects/red', self.make_position_callback('red'), 10)
        self.create_subscription(PointStamped, '/detected_objects/blue', self.make_position_callback('blue'), 10)
        self.create_subscription(String, '/voice_command', self.command_callback, 10)

        self.action_client = ActionClient(self, ArduinobotTask, 'task_server')
        self.action_client.wait_for_server()

    def make_position_callback(self, color):
        def callback(msg):
            self.latest_positions[color] = (msg.point.x, msg.point.y, msg.point.z)
        return callback

    def command_callback(self, msg):
        text = msg.data.lower()

        target_color = None
        if 'red' in text:
            target_color = 'red'
        elif 'blue' in text:
            target_color = 'blue'

        if target_color is None:
            self.get_logger().info('No known color found in command')
            return

        if target_color not in self.latest_positions:
            self.get_logger().info(f'No detection yet for {target_color}')
            return

        x, y, z = self.latest_positions[target_color]

        goal = ArduinobotTask.Goal()
        goal.task_number = 3
        goal.target_x = x
        goal.target_y = y
        goal.target_z = z

        self.action_client.send_goal_async(goal)
        self.get_logger().info(f'Sent pick goal for {target_color} at ({x:.3f}, {y:.3f}, {z:.3f})')

def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()