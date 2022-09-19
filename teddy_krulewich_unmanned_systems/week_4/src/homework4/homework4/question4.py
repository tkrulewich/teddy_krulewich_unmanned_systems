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


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.last_error = 0
        self.integral = 0

        self.output = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        self.output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.last_error = error

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)


        self.start_time = self.get_clock().now().nanoseconds
        self.done = False

        self.state_records = { 'cmd_vel_linear': [], 'cmd_vel_angular': [], 'x': [], 'y': [], 'theta': [] }

        self.desired_theta = None
        self.desired_x = None
        self.desired_y = None

        self.current_theta = None
        self.current_x = None
        self.current_y = None

        self.theta_controller = PID(0.5, 0.0, 0.0)


    
    def add_move_command(self, linear, angular, duration):
        self.move_commands.append(TurtleBotController.MoveCommand(linear, angular, duration))
    
    def timer_callback(self):
        if self.done:
            return

        if self.desired_theta is None or self.desired_x is None or self.desired_y is None:
            return
        
        if self.current_theta is None or self.current_x is None or self.current_y is None:
            return

        if math.abs(self.current_theta - self.desired_theta) < 0.01:
            self.done = True
            return
        
        twist = Twist()

        twist.linear.x = 0.15

        twist.angular.z = self.theta_controller.update(self.desired_theta - self.current_theta, 0.1)

        self.cmd_vel_publisher.publish(twist)

        time = self.get_clock().now().nanoseconds

        self.state_records['cmd_vel_linear'].append(twist.linear.x)
        self.state_records['cmd_vel_angular'].append(twist.angular.z)
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]

            
    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]

        self.state_records['x'].append(self.current_x)
        self.state_records['y'].append(self.current_y)
        self.state_records['theta'].append(self.current_theta)

def main(args=None):
    rclpy.init(args=args)

    turtlebot_controller = TurtleBotController()

    turtlebot_controller.desired_theta = math.pi / 2
    turtlebot_controller.desired_x = 100
    turtlebot_controller.desired_y = 100

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


