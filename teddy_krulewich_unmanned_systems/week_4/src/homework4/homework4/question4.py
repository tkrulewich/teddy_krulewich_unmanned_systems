import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import numpy as np
import math

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

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
    """
    Simple PID controller for a single variable
    """
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.last_error = 0
        self.integral = 0

        self.output = 0

    def update(self, error, dt):
        """
        Update the PID controller using new error value
        """
        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        self.output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.last_error = error

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # create a publisher to send velocity commands to the turtlebot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # create a subscriber to read sensor data
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)


        # store the time the node was started
        self.start_time = self.get_clock().now().nanoseconds
        self.last_update = self.start_time

        # the turtle bot will continue updating until this is set to true
        self.done = False

        self.state_records = { 'cmd_vel_linear': [], 'cmd_vel_angular': [], 'x': [], 'y': [], 'theta': [] }


        self.desired_theta = None
        self.current_theta = None

        self.theta_controller = PID(7, 0.0, 1.0)
                
    def odom_callback(self, msg):
        # if we are done dont execute anything else
        if self.done:
            return

        # if we have not set a desired heading 
        if self.desired_theta is None:
            self.done = True
        
        # get current time and time elapsed since the node was started
        time = self.get_clock().now().nanoseconds
        time_elapsed = time - self.start_time

        # get the dt in seconds since the last update
        dt = (time - self.last_update)
        

        # read sensor data to get position and heading
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]

        # store sensor data for logging and plotting
        self.state_records['x'].append((time_elapsed, self.current_x))
        self.state_records['y'].append((time_elapsed, self.current_y))
        self.state_records['theta'].append((time_elapsed, self.current_theta))

        # create a Twist for the velocity command
        twist = Twist()
        # move forward at 0.15 m/s
        twist.linear.x = 0.15


        # if more than 3 seconds have elapsed, stop the turtlebot
        if time_elapsed > 3000000000: 
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.done = True
            return
        
        # run a PID controller using the error in heading
        self.theta_controller.update(self.desired_theta - self.current_theta, dt)
        
        # set the angular velocity to the output of the PID controller
        twist.angular.z = self.theta_controller.output

        twist.angular.z = clamp(twist.angular.z, -2.84, 2.84)

        # publish the velocity command to the turtlebot
        self.cmd_vel_publisher.publish(twist)

        # store the velocity command for logging and plotting
        self.state_records['cmd_vel_linear'].append((time, twist.linear.x))
        self.state_records['cmd_vel_angular'].append((time, twist.angular.z))

        self.last_update = time


def main(args=None):
    rclpy.init(args=args)

    turtlebot_controller = TurtleBotController()

    turtlebot_controller.desired_theta = math.pi / 2

    while not turtlebot_controller.done:
        rclpy.spin_once(turtlebot_controller)
    

    plt.plot([theta[0] / 1000000000 for theta in turtlebot_controller.state_records['theta']], [math.degrees(theta[1]) for theta in turtlebot_controller.state_records['theta']], label='Theta Actual')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')

    plt.plot([theta[0] / 1000000000 for theta in turtlebot_controller.state_records['theta']], [90.0 for theta in turtlebot_controller.state_records['theta']], label='Theta Desired')
    
    # plot the 10% and 90% lines of the commanded heading
    plt.axhline(y=81, color="black", linestyle="--")
    plt.axhline(y=10, color="black", linestyle="--")

    plt.legend()
    plt.show()
    

    turtlebot_controller.destroy_node()
    rclpy.shutdown()
    turtlebot_controller.desired_theta = math.pi / 2

    while not turtlebot_controller.done:
        rclpy.spin_once(turtlebot_controller)
    

    plt.plot([theta[0] / 1000000000 for theta in turtlebot_controller.state_records['theta']], [math.degrees(theta[1]) for theta in turtlebot_controller.state_records['theta']], label='Theta Actual')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')

    plt.plot([theta[0] / 1000000000 for theta in turtlebot_controller.state_records['theta']], [90.0 for theta in turtlebot_controller.state_records['theta']], label='Theta Desired')
    
    # plot the 10% and 90% lines of the commanded heading
    plt.axhline(y=81, color="black", linestyle="--")
    plt.axhline(y=10, color="black", linestyle="--")

    plt.legend()
    plt.show()
    

    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


