import matplotlib.pyplot as plt
import csv

if __name__ == '__main__':

    pursuer_x = []
    pursuer_y = []

    evader_x = []
    evader_y = []

    with open('persuer_path.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)

        next(reader)
        for row in reader:
            pursuer_x.append(float(row[0]))
            pursuer_y.append(float(row[1]))
    

    with open('Evader_pose_log.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)

        next(reader)
        for row in reader:
            evader_x.append(float(row[1]))
            evader_y.append(float(row[2]))

    plt.plot(pursuer_x, pursuer_y, color='blue', linewidth=2, label='Persuer Path')
    plt.plot(evader_y, evader_x, color='red', linewidth=2, label='Evader Path')

    plt.legend()
    plt.show()