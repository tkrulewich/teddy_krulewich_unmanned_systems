from homework6.submodules.TurtleBot import TurtleBotController, clamp
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from geometry_msgs.msg import Twist

class PursuerTurtleBot(TurtleBotController):
    def __init__(self, namespace="", pnGAin=1.5):
        super().__init__(namespace)

        # subscribe to lider topic
        self.lidear_subscriber = self.create_subscription(LaserScan, namespace + '/scan', self.lidear_callback, 10)

        self.evader_theta = None
        self.evader_theta_prev = None

        # the gain for the proportional navigation controller
        self.PNGain = pnGAin

        # clocks to for synchronizing dt in the odom and lidar callbacks
        self.last_update_odom = self.get_clock().now().nanoseconds
        self.last_update_lidar = self.get_clock().now().nanoseconds


    def odom_callback(self, msg):
        # get current time, time elapsed, and delta time
        time = self.get_clock().now().nanoseconds
        time_elapsed = time - self.start_time
        dt = time - self.last_update_odom

        self.last_update_odom = time

        # read sensor data to get position and heading
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # store the sensor data and desired heading for logging and plotting
        self.state_records['x'].append((time_elapsed, self.current_x))
        self.state_records['y'].append((time_elapsed, self.current_y))

        # if we crashed into something stop the turtlebot
        if abs(msg.pose.pose.orientation.y) > 0.03:
            self.done = True
    
    def lidear_callback(self, msg):
        # get current time, time elapsed, and delta time
        time = self.get_clock().now().nanoseconds
        time_elapsed = time - self.start_time
        dt = time - self.last_update_lidar

        self.last_update_lidar = time

        # change dt to seconds instead of nano seconds
        dt /= 1000000000

        # get the angle of the closest object
        self.evader_theta = np.argmin(msg.ranges)

        # if no closest object found exit
        if self.evader_theta is None:
            return
        
        # change angle to radians
        self.evader_theta = math.radians(self.evader_theta)

        # wrap angle to be between -pi and pi
        if self.evader_theta > math.pi:
            self.evader_theta -= 2 * math.pi

        # if this is the first iteration, then there is no previous, set previous to current and end
        if self.evader_theta_prev is None:
            self.evader_theta_prev = self.evader_theta
            return

        # calculate theta dot based on previous and current theta
        self.theta_dot = (self.evader_theta - self.evader_theta_prev) / dt
        self.evader_theta_prev = self.evader_theta
        

        # find turn rate based on theta dot and gain
        turn_rate = self.PNGain * abs(self.theta_dot)

        # find the direction to turn
        turn_direction = 1.0 if self.evader_theta > 0.0 else -1.0

        # send the velocity command
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = turn_direction * turn_rate

        twist.angular.z = clamp(twist.angular.z, -2.84, 2.84)

        self.cmd_vel_publisher.publish(twist)

