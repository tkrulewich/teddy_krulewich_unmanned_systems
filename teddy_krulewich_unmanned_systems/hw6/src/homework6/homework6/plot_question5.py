import matplotlib.pyplot as plt
import csv

if __name__ == '__main__':

    pursuer_x = []
    pursuer_y = []

    evader_x = []
    evader_y = []

    i_was_pursuing = True

    pursuer_filename = "pursuer_path.csv" if i_was_pursuing else "Pursuer_pose_log.csv"
    evader_filename = "evader_path.csv" if not i_was_pursuing else "Evader_pose_log.csv"
    

    with open(pursuer_filename, 'r') as csvfile:
        reader = csv.reader(csvfile)

        next(reader)
        for row in reader:
            pursuer_x.append(float(row[1]))
            pursuer_y.append(float(row[2]))
    

    with open(evader_filename, 'r') as csvfile:
        reader = csv.reader(csvfile)

        next(reader)
        for row in reader:
            evader_x.append(float(row[1]))
            evader_y.append(float(row[2]))

    plt.plot(pursuer_x, pursuer_y, color='blue', linewidth=2, label='Persuer Path')
    plt.plot(evader_x, evader_y, color='red', linewidth=2, label='Evader Path')

    plt.legend()
    plt.show()