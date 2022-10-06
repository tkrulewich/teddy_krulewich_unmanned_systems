from homework5.submodules.TurtleBot import TurtleBotController
from homework5.submodules.PathPlanning import *
import rclpy

import matplotlib.pyplot as plt

def main(args=None):
    grid = Grid(0, 15, 0, 15, 0.5)

    obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 8, 8, 8, 2, 3, 4, 5, 6, 7,
        9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5, 5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]

    obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6,
        6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]

    for x, y in zip(obstacle_x, obstacle_y):
        grid.add_obstacle(Obstacle(x, y, 0.49))
    
    grid.inflate(0.5)

    start = grid.nodes[(1,1)]
    end = grid.nodes[(7, 13)]

    x, y = grid.a_star(start,end)

    x.reverse()
    y.reverse()

    grid.draw(show_obstacles = False)

    plt.plot(x, y, color='red', linewidth=2, label='A* Path')
    
    rclpy.init(args=args)

    turtlebot_controller = TurtleBotController()

    turtlebot_controller.done = False
    
    for i in range(len(x)):
        turtlebot_controller.add_waypoint(x[i], y[i])
    
    while not turtlebot_controller.done:
        rclpy.spin_once(turtlebot_controller)
    

    plt.plot([x[1] for x in turtlebot_controller.state_records['x']],[y[1] for y in turtlebot_controller.state_records['y']], color='blue', linewidth=2, label='TurtleBot Actual Path')
    
    plt.legend()
    plt.show()
    
    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


