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

        self.theta_controller = PID(1, 0.5, 0.0)


    
    def add_move_command(self, linear, angular, duration):
        self.move_commands.append(TurtleBotController.MoveCommand(linear, angular, duration))
    
    def timer_callback(self):
        if self.done:
            return

        if self.desired_theta is None or self.desired_x is None or self.desired_y is None:
            return
        
        if self.current_theta is None or self.current_x is None or self.current_y is None:
            return

        time = self.get_clock().now().nanoseconds

        twist = Twist()
        twist.linear.x = 0.15

        if time - self.start_time > 30000000000:
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.done = True
            return
        

        self.theta_controller.update(self.desired_theta - self.current_theta, 0.1)
        twist.angular.z = self.theta_controller.output

        self.cmd_vel_publisher.publish(twist)

        self.state_records['cmd_vel_linear'].append((time, twist.linear.x))
        self.state_records['cmd_vel_angular'].append((time, twist.angular.z))
                
    def odom_callback(self, msg):
        time = self.get_clock().now().nanoseconds
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]

        self.state_records['x'].append((time - self.start_time, self.current_x))
        self.state_records['y'].append((time - self.start_time, self.current_y))
        self.state_records['theta'].append((time - self.start_time, self.current_theta))


def main(args=None):
    rclpy.init(args=args)

    turtlebot_controller = TurtleBotController()

    turtlebot_controller.desired_theta = math.pi / 2
    turtlebot_controller.desired_x = 100
    turtlebot_controller.desired_y = 100

    while not turtlebot_controller.done:
        rclpy.spin_once(turtlebot_controller)
    

    plt.plot([theta[0] / 1000000000 for theta in turtlebot_controller.state_records['theta']], [theta[1] for theta in turtlebot_controller.state_records['theta']], label='Theta Actual')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')

    plt.plot([theta[0] / 1000000000 for theta in turtlebot_controller.state_records['theta']], [math.pi / 2 for theta in turtlebot_controller.state_records['theta']], label='Theta Desired')
    
    plt.legend()
    plt.show()
    

    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


