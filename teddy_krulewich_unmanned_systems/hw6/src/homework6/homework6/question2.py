from homework6.submodules.TurtleBot import TurtleBotController
from homework6.submodules.PursuerTurtleBot import PursuerTurtleBot
import rclpy

import matplotlib.pyplot as plt


def main(args=None):    
    rclpy.init(args=args)

    evader = TurtleBotController("turtle")
    
    # 10 % original value
    # persuer = PursuerTurtleBot(pnGAin=.15)

    # 100 % original value
    # persuer = PursuerTurtleBot(pnGAin=1.5)

    # 1000 % original value
    persuer = PursuerTurtleBot(pnGAin=150)
    
    evader.add_waypoint(9, 9)
    
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


