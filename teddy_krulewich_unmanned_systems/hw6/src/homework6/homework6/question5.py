from homework6.submodules.TurtleBot import TurtleBotController
from homework6.submodules.PersuerTurtleBot import PersuerTurtleBot
import rclpy

import matplotlib.pyplot as plt
import csv

def main(args=None):    
    rclpy.init(args=args)

    persuer = PersuerTurtleBot()
    
    
    while not persuer.done:
        rclpy.spin_once(persuer)

    persuer.destroy_node()

    rclpy.shutdown()

    # save to csv
    with open('persuer_path.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y'])
        for i in range(len(persuer.state_records['x'])):
            writer.writerow([persuer.state_records['x'][i][1], persuer.state_records['y'][i][1]])

if __name__ == '__main__':
    main()


