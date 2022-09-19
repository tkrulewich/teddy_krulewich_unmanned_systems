import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import numpy as np
import math

def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


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
        self.done = False

        self.state_records = { 'cmd_vel_linear': [], 'cmd_vel_angular': [], 'x': [], 'y': [], 'theta': [] }


    
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
                    self.done = True
                    return
            
            self.state_records['cmd_vel_linear'].append((time - self.start_time, self.move_commands[0].twist.linear.x))
            self.state_records['cmd_vel_angular'].append((time - self.start_time, self.move_commands[0].twist.angular.z))

            self.cmd_vel_publisher.publish(self.move_commands[0].twist)
            
    def odom_callback(self, msg):
        time = self.get_clock().now().nanoseconds
        self.get_logger().info('Odometry: {}'.format(msg.pose.pose.position))

        self.state_records['x'].append((time - self.start_time, msg.pose.pose.position.x))
        self.state_records['y'].append((time - self.start_time, msg.pose.pose.position.y))
        theta = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]
        self.state_records['theta'].append((time - self.start_time, theta))

def main(args=None):
    rclpy.init(args=args)

    turtlebot_controller = TurtleBotController()

    turtlebot_controller.add_move_command(1.5, 0.0, 5000000000)
    turtlebot_controller.add_move_command(0.0, -0.15, 2000000000)
    turtlebot_controller.add_move_command(1.5, 0.0, 5000000000)

    while not turtlebot_controller.done:
        rclpy.spin_once(turtlebot_controller)
    
    fig, ax = plt.subplots(3, 1)

    ax[0].plot([x[0] / 1000000000 for x in turtlebot_controller.state_records['cmd_vel_linear']], [x[1] for x in turtlebot_controller.state_records['cmd_vel_linear']], label='linear')
    ax[0].set_xlabel('Time (ns)')
    ax[0].set_ylabel('Linear Velocity (m/s)')

    ax[1].plot([x[0] / 1000000000 for x in turtlebot_controller.state_records['cmd_vel_angular']], [x[1] for x in turtlebot_controller.state_records['cmd_vel_angular']], label='angular')
    ax[1].set_xlabel('Time (ns)')
    ax[1].set_ylabel('Angular Velocity (rad/s)')

    ax[2].plot([x[1] for x in turtlebot_controller.state_records['x']], [y[1] for y in turtlebot_controller.state_records['y']], label='y')
    ax[2].set_xlabel('X Position (m)')
    ax[2].set_ylabel('Y Position (m)')
    

    plt.show()
    

    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


