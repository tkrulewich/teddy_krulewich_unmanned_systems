import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import numpy as np

class TurtleBotController(Node):
    class MoveCommand:
        def __init__(self, linear, angular, duration):
            self.twist = Twist()
            self.twist.linear.x = linear
            self.twist.angular.z = angular
            self.duration = duration

    def __init__(self):
        super().__init__('turtlebot_controller')

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.move_commands = []
        self.start_time = self.get_clock().now().nanoseconds
        self.time_command_started = self.start_time

        self.data = { 'odom': [], 'cmd_vel': [] }
    
    def add_move_command(self, linear, angular, duration):
        self.move_commands.append(TurtleBotController.MoveCommand(linear, angular, duration))
    
    def timer_callback(self):
        time = self.get_clock().now().nanoseconds
        if len(self.move_commands) > 0:
            if time - self.time_command_started > self.move_commands[0].duration:
                self.move_commands.pop(0)
                self.time_command_started = self.get_clock().now().nanoseconds

                if len(self.move_commands) == 0:
                    self.cmd_vel_publisher.publish(Twist())
                    self.get_logger().info('No more commands')
                    self.destroy_timer(self.timer)
                    self.destroy_subscription(self.odom_subscriber)

                    self.plot_data()

                    return
            
            self.data['cmd_vel'].append((time, self.move_commands[0].twist))
            self.cmd_vel_publisher.publish(self.move_commands[0].twist)
            

    
    def odom_callback(self, msg):
        time = self.get_clock().now().nanoseconds
        self.data['odom'].append((time, msg))

    def plot_data(self):
        # x_list = []
        # y_list = []

        # for odom in self.data['odom']:
        #     x_list.append((odom[0] - self.start_time) / 1000000000)
        #     y_list.append(odom[1].pose.pose.position.x)

        # plt.plot(x_list, y_list)
        # plt.show()

        # x_list = []
        # y_list = []

        # for odom in self.data['odom']:
        #     x_list.append((odom[0] - self.start_time) / 1000000000)
        #     y_list.append(odom[1].pose.pose.position.y)

        # plt.plot(x_list, y_list)
        # plt.show()

        fig, ax = plt.subplots(2, 1)
    
        x_list = []
        y_list = []

        for vel in self.data['cmd_vel']:
            x_list.append((vel[0] - self.start_time) / 1000000000)
            y_list.append(vel[1].linear.x)

        ax[0].plot(x_list, y_list)
        ax[0].set_ylabel('Commanded Linear Velocity (m/s)')
        ax[0].set_xlabel('Time (s)')

        x_list = []
        y_list = []

        for odom in self.data['odom']:
            x_list.append((odom[0] - self.start_time) / 1000000000)
            y_list.append(odom[1].twist.twist.linear.x)
        
        ax[1].plot(x_list, y_list)
        ax[1].set_ylabel('Measured Linear Velocity (m/s)')
        ax[1].set_xlabel('Time (s)')
        plt.show()



def main(args=None):
    rclpy.init(args=args)

    turtlebot_controller = TurtleBotController()

    turtlebot_controller.add_move_command(1.5, 0.0, 5000000000)
    turtlebot_controller.add_move_command(0.0, -0.15, 2000000000)
    turtlebot_controller.add_move_command(1.5, 0.0, 5000000000)

    rclpy.spin(turtlebot_controller)

    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


