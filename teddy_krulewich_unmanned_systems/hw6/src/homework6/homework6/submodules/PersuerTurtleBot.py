from homework6.submodules.TurtleBot import TurtleBotController, clamp
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from geometry_msgs.msg import Twist

class PersuerTurtleBot(TurtleBotController):
    def __init__(self, namespace="", pnGAin=1.5):
        super().__init__(namespace)

        self.lidear_subscriber = self.create_subscription(LaserScan, namespace + '/scan', self.lidear_callback, 10)

        self.evader_theta = None
        self.evader_theta_prev = None

        self.PNGain = pnGAin

        self.last_update_odom = self.get_clock().now().nanoseconds
        self.last_update_lidar = self.get_clock().now().nanoseconds

        self.prev_theta_dot = None

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

        if abs(msg.pose.pose.orientation.y) > 0.03:
            self.done = True
    
    def lidear_callback(self, msg):
        # get current time, time elapsed, and delta time
        time = self.get_clock().now().nanoseconds
        time_elapsed = time - self.start_time
        dt = time - self.last_update_lidar

        self.last_update_lidar = time

        dt /= 1000000000

        self.evader_theta = np.argmin(msg.ranges)

        if self.evader_theta is None:
            return
        
        self.evader_theta = math.radians(self.evader_theta)

        if self.evader_theta > math.pi:
            self.evader_theta -= 2 * math.pi

        if self.evader_theta_prev is None:
            self.evader_theta_prev = self.evader_theta
            return

        
        self.theta_dot = (self.evader_theta - self.evader_theta_prev) / dt
        self.evader_theta_prev = self.evader_theta
        

        if self.prev_theta_dot is None:
            self.prev_theta_dot = self.theta_dot
        
        

        turn_rate = self.PNGain * abs(self.theta_dot)

        turn_direction = 1.0 if self.evader_theta > 0.0 else -1.0

        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = turn_direction * turn_rate

        twist.angular.z = clamp(twist.angular.z, -2.84, 2.84)

        self.cmd_vel_publisher.publish(twist)

