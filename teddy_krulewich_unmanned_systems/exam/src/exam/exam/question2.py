from exam.submodules.TurtleBot import TurtleBotController
from exam.submodules.PursuerTurtleBot import PursuerTurtleBot
from exam.submodules.EvaderTurtleBot import EvaderTurtleBot
from exam.submodules.PathPlanning import Grid, Obstacle
import rclpy

import matplotlib.pyplot as plt

import random

def main(args=None):    
    rclpy.init(args=args)

    grid = Grid(0, 15, 0, 15, 0.5)

    evader = TurtleBotController("evader")
    persuer = PursuerTurtleBot("pursuer", pnGAin=5.0)

    obstacle_list = [(5, 0),(5, 1),(5, 2),(5, 3),(5, 4),(0, 5),(1, 4),(2, 3),(3, 2),(3, 3)]

    for obstacle in obstacle_list:
        grid.add_obstacle(Obstacle(obstacle[0], obstacle[1], 0.49))
    
    grid.inflate(0.5)

    start = grid.nodes[(2,1)]
    end = grid.nodes[(7, 2)]

    x, y = grid.a_star(start,end)

    x.reverse()
    y.reverse()

    for i in range(len(x)):
        evader.add_waypoint(x[i], y[i])
    


    
    while not evader.done:
        rclpy.spin_once(evader)
        rclpy.spin_once(persuer)

    evader.destroy_node()
    persuer.destroy_node()

    rclpy.shutdown()

    # plot evader path
    plt.plot([x[1] for x in evader.state_records['x']],[y[1] for y in evader.state_records['y']], color='red', linewidth=2, label='Evader Path')
    # plot pursuer path
    plt.plot([x[1] for x in persuer.state_records['x']],[y[1] for y in persuer.state_records['y']], color='blue', linewidth=2, label='Persuer Path')
    
    # plot desired path of evader
    plt.plot(x, y, color='green', linewidth=2, label='Desired Path')

    
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()


