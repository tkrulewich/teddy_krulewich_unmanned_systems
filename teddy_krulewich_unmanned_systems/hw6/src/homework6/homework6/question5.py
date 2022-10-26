from homework6.submodules.TurtleBot import TurtleBotController
from homework6.submodules.PursuerTurtleBot import PursuerTurtleBot
from homework6.submodules.EvaderTurtleBot import EvaderTurtleBot
import rclpy

import matplotlib.pyplot as plt
import csv

def main(args=None):    
    rclpy.init(args=args)

    pursuing = False
    turtlebot = None

    if pursuing:
        turtlebot = PursuerTurtleBot()
    else:
        turtlebot = EvaderTurtleBot("turtle")
    
    while not turtlebot.done:
        rclpy.spin_once(turtlebot)

    turtlebot.destroy_node()

    rclpy.shutdown()

    filename = "pursuer_path.csv" if pursuing else "evader_path.csv"

    # save to csv
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['time','x', 'y'])
        for i in range(len(turtlebot.state_records['x'])):
            writer.writerow([turtlebot.state_records['x'][i][0],turtlebot.state_records['x'][i][1], turtlebot.state_records['y'][i][1]])

if __name__ == '__main__':
    main()


