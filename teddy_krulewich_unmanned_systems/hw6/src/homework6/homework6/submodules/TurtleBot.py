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
    class Waypoint:
        """
        A waypoint that the turtlebot will try to reach
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
    
    def __init__(self, namespace=""):
        super().__init__('turtlebot_controller' + namespace)

        # create a publisher to send velocity commands to the turtlebot
        self.cmd_vel_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        # create a subscriber to read sensor data
        self.odom_subscriber = self.create_subscription(Odometry, namespace + '/odom', self.odom_callback, 10)


        # store the time the node was started
        self.start_time = self.get_clock().now().nanoseconds
        self.last_update = self.start_time

        # the turtle bot will continue updating until this is set to true
        self.done = False

        # store the state for logging and plotting
        self.state_records = { 'cmd_vel_linear': [], 'cmd_vel_angular': [], 'x': [], 'y': [], 'theta': [], 'theta_des': [] }

        # current position and location of the turtlebot
        self.current_theta = None
        self.current_x = None
        self.current_y = None

        # create a PID controller for angular velocity
        self.theta_controller = PID(7, 0.0, 0.0)

        # the list of waypoints in order that the turtlebot will try to reach
        self.waypoints = []


    
    def add_waypoint(self, x, y):
        self.waypoints.append(self.Waypoint(x, y))
                
    def odom_callback(self, msg):
        # get current time, time elapsed, and delta time
        time = self.get_clock().now().nanoseconds
        time_elapsed = time - self.start_time
        dt = time - self.last_update
        

        self.last_update = time

        # if there are no more waypoings, stop the turtlebot
        if len(self.waypoints) == 0 or time - self.start_time > 30000000000:
            self.done = True
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            return

        # read sensor data to get position and heading
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]

        # get the current waypoing (top of list)
        waypoint = self.waypoints[0]
        # find its distance from the turtlebot
        distance = math.sqrt((waypoint.x - self.current_x)**2 + (waypoint.y - self.current_y)**2)
        
        # if the turtlebot is close enough to the waypoint, move on
        if distance < 0.1:
            self.waypoints.pop(0)
            return

        
        # find the angle between current position and waypoint
        self.desired_theta = math.atan2(waypoint.y - self.current_y, waypoint.x - self.current_x)

        # adjust desired angle so that it turns the shortest way
        if self.desired_theta - self.current_theta > math.pi:
            self.desired_theta = self.desired_theta - 2 * math.pi
        elif self.desired_theta - self.current_theta < -math.pi:
            self.desired_theta = self.desired_theta + 2 * math.pi

        # store the sensor data and desired heading for logging and plotting
        self.state_records['x'].append((time_elapsed, self.current_x))
        self.state_records['y'].append((time_elapsed, self.current_y))
        self.state_records['theta'].append((time_elapsed, self.current_theta))
        self.state_records['theta_des'].append((time_elapsed, self.desired_theta))

        # set the translational velocity to 0.15 m/s
        twist = Twist()
        twist.linear.x = 0.1

        # use PID controller to set the angular velocity
        self.theta_controller.update(self.desired_theta - self.current_theta, dt)
        twist.angular.z = self.theta_controller.output

        # cap the angular velocity at 2.84 rad/s
        twist.angular.z = clamp(twist.angular.z, -2.84, 2.84)

        # publish the velocity command to the turtlebot
        self.cmd_vel_publisher.publish(twist)
        
        # store the velocity command for logging and plotting
        self.state_records['cmd_vel_linear'].append((time_elapsed, twist.linear.x))
        self.state_records['cmd_vel_angular'].append((time_elapsed, twist.angular.z))
