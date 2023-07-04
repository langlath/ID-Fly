
import os
import numpy as np
import matplotlib.pyplot as plt


def get_array_traj(file):
    """get a file and synchronizes trajectories in it

    Args:
        file (file): the open file where trajectories are

    Returns:
        float[n, 5]: array of trajectories. First column is time, next columns go 2 by 2 with coordinates at each time.
    """
    # we create a list of the trajectories
    lis_traj = []
    for line in file : 
        line = line.split()
        # if we have a new trajectory we create a new one in the list
        if "Trajectoire" in line : 
            lis_traj.append([])
        # if the line is coordinates with time we add it to the last trajectory
        elif len(lis_traj) > 0 and len(line) == 3:
            lis_traj[-1].append([float(i.replace(",", ".")) for i in line])

    # if we have trajectories we store them in an array
    if len(lis_traj) > 0 :
        # first we make sure we have data for every trajectory at every time
        array_traj = [np.array(traj) for traj in lis_traj]
        for i in range(len(lis_traj)) :
            traj = lis_traj[i]
            for j in range(len(traj) - 1, -1, -1):
                time = traj[j][0]
                synchro = True
                for k in range(len(lis_traj)):
                    # if we do not have data for every trajectory at selected time we delete it
                    if k != i and (not time in array_traj[k][:, 0]):
                        synchro = False
                if not synchro : 
                    del traj[j]

        # now that trajectories are synchronized we put them in a common array
        array_traj = np.ones((len(lis_traj[0]), 2 * len(lis_traj) + 1))
        for i in range(len(lis_traj[0])):
            array_traj[i, 0] = lis_traj[0][i][0]
            for j in range(len(lis_traj)):
                array_traj[i, 2 * j + 1] = lis_traj[j][i][1]
                array_traj[i, 2 * j + 2] = lis_traj[j][i][2]
        return array_traj
    else : 
        return(np.array([]))

def diff_angles(a1, a2):
    possibles = np.array([a2 - a1, a2 + np.pi - a1, a2 - (np.pi + a1)])
    return possibles[np.argmin(np.abs(possibles))]

if __name__ == "__main__" :
    print("Your current path is :\n", os.getcwd(), "\nPlease make sure you are targeting the right folder.")
    print("Please enter the name of the target file :")
    filename = input()
    file = open(filename, "r")
    arr_traj = get_array_traj(file)
    file.close()

    lis_angles = []
    for i in range(len(arr_traj)):
        lis_times = arr_traj[:, 0]
        lis_angles.append(np.arctan2((arr_traj[i, 4] - arr_traj[i, 2]), (arr_traj[i, 3] - arr_traj[i, 1])))
    # plt.plot(lis_times, lis_angles)
    # plt.show()

    lis_speed = []
    for i in range(1, len(lis_angles)):
        lis_speed.append(diff_angles(lis_angles[i - 1], lis_angles[i]) / (lis_times[i] - lis_times[i - 1]))
    plt.plot(lis_times[1:], lis_speed)
    plt.title("Angular speed over time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular speed (rad/s)")
    plt.pause(2)

    lis_filename = filename.split(".")
    filename2 = lis_filename[0] + "_converted." + lis_filename[1]
    file_write = open(filename2, "w")
    file_write.write("rotation speed from file " + filename + "\n\n")
    for i in range(len(lis_speed)):
        file_write.write(str(lis_times[i + 1]) + " " + str(lis_speed[i]) + "\n")
    file_write.close()
    print("file successfully written")
    
    
