from homework6.submodules.TurtleBot import TurtleBotController
from homework6.submodules.PursuerTurtleBot import PursuerTurtleBot
import rclpy

import matplotlib.pyplot as plt

import random

def main(args=None):    
    rclpy.init(args=args)

    evader = TurtleBotController("turtle")
    persuer = PursuerTurtleBot()

    x = 2
    y = 2

    waypoints = []
    
    for i in range(100):
        x += random.random() * 1.0 - 0.5
        y += random.random() * 1.0 - 0.5

        evader.add_waypoint(x, y)
        waypoints.append((x, y))
    
    while not evader.done:
        rclpy.spin_once(evader)
        rclpy.spin_once(persuer)

    evader.destroy_node()
    persuer.destroy_node()

    rclpy.shutdown()

    plt.plot([x[1] for x in evader.state_records['x']],[y[1] for y in evader.state_records['y']], color='red', linewidth=2, label='Evader Path')
    plt.plot([x[1] for x in persuer.state_records['x']],[y[1] for y in persuer.state_records['y']], color='blue', linewidth=2, label='Persuer Path')
    
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()


