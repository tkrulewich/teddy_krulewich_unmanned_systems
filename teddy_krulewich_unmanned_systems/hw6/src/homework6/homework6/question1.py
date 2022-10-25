from homework6.submodules.TurtleBot import TurtleBotController
import rclpy

import matplotlib.pyplot as plt

def main(args=None):    
    rclpy.init(args=args)

    turtlebot_controller = TurtleBotController()

    turtlebot_controller.done = False
    
    turtlebot_controller.add_waypoint(9, 9)
    
    while not turtlebot_controller.done:
        rclpy.spin_once(turtlebot_controller)

    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


