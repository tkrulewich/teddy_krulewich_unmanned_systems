from homework6.submodules.TurtleBot import TurtleBotController, clamp, euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from geometry_msgs.msg import Twist

from scipy.spatial.distance import cdist 

#Function to implement steps given in previous section
def kmeans(x,k, no_of_iterations):
    idx = np.random.choice(len(x), k, replace=False)
    #Randomly choosing Centroids 
    centroids = x[idx, :] #Step 1
     
    #finding the distance between centroids and all the data points
    distances = cdist(x, centroids ,'euclidean') #Step 2
     
    #Centroid with the minimum Distance
    points = np.array([np.argmin(i) for i in distances]) #Step 3
     
    #Repeating the above steps for a defined number of iterations
    #Step 4
    for _ in range(no_of_iterations): 
        centroids = []
        for idx in range(k):
            #Updating Centroids by taking mean of Cluster it belongs to
            temp_cent = x[points==idx].mean(axis=0) 
            centroids.append(temp_cent)
 
        centroids = np.vstack(centroids) #Updated Centroids 
         
        distances = cdist(x, centroids ,'euclidean')
        points = np.array([np.argmin(i) for i in distances])
         
    return centroids

class PursuerTurtleBot(TurtleBotController):
    def __init__(self, namespace="", pnGAin=1.5):
        super().__init__(namespace)

        # subscribe to lider topic
        self.lidear_subscriber = self.create_subscription(LaserScan, namespace + '/scan', self.lidear_callback, 10)

        self.evader_theta = None
        self.evader_theta_prev = None

        self.prev_target = None

        self.evader_prev_x = None
        self.evader_prev_y = None

        # the gain for the proportional navigation controller
        self.PNGain = pnGAin

        self.last_distance = None

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
        self.current_theta = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]



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

        if self.current_x is None or self.current_y is None:
            return

        detected_points = []
        detected_angles = []
        detected_distancs = []

        closest_distance = math.inf
        closest_angle = None

        for i in range(len(msg.ranges)):
            consecutive = 0
            if msg.ranges[i] < math.inf:
                angle = math.radians(i / 5.0)
                x = msg.ranges[i] * math.cos(math.radians(angle))
                y = msg.ranges[i] * math.sin(math.radians(angle))
                dist = msg.ranges[i]

                if dist < closest_distance:
                    for j in range(-5, 5):
                        index = i + j
                        if index < 0:
                            index = len(msg.ranges) + index
                        elif index >= len(msg.ranges):
                            index = index - len(msg.ranges)

                        angle2 = math.radians(index / 5.0)
                        x2 = msg.ranges[index] * math.cos(math.radians(angle2))
                        y2 = msg.ranges[index] * math.sin(math.radians(angle2))

                        distbetween = math.sqrt((x2 - x) ** 2 + (y2 - y) ** 2)

                        if distbetween < 0.2:
                            consecutive += 1
                
                    if consecutive < 10:
                        closest_distance = dist
                        closest_angle = angle
        
        self.evader_theta = closest_angle

        if self.evader_theta is None:
            return

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


        

        # # find turn rate based on theta dot and gain
        # turn_rate = self.PNGain * abs(self.theta_dot)

        # # find the direction to turn
        # turn_direction = 1.0 if self.evader_theta > 0.0 else -1.0

        # send the velocity command

        self.desired_theta = self.PNGain * self.theta_dot

        
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = self.desired_theta

        twist.angular.z = clamp(twist.angular.z, -2.84, 2.84)

        self.cmd_vel_publisher.publish(twist)

