import os
import math
import argparse

import numpy as np
from matplotlib import pyplot as plt

SHOW_DEGREE = True
OFFSET = 0

def calculateRMSE(time_pre: np.array, time_gt: np.array, state_pre: np.array, state_gt: np.array):
    t_min = np.min(time_pre)
    t_max = np.max(time_pre)

    sum_squere_error = 0.0
    count = 0
    for i, t in enumerate(time_gt):
        if t < t_min or t > t_max:
            continue

        state_inter = np.interp(t, time_pre, state_pre)
        sum_squere_error += pow(state_inter - state_gt[i], 2)
        count += 1
    return pow(sum_squere_error/count, 0.5)


def parseFile(path: str, filename: str):

    timestamp_list = []
    radian_list = []
    degree_list = []

    with open(os.path.join(path, filename), "r") as f:
        for line in f:
            text = line.split(" ")
            assert len(text) == 2

            timestamp_list.append(float(text[0]))
            radian_list.append(float(text[1]))
            degree_list.append(float(text[1]) * 180 / math.pi)

    print(
        f"Finish reading {os.path.basename(path)} with {len(timestamp_list)} data")
    return timestamp_list, radian_list, degree_list


def draw(root: str):
    m_t, m_r, m_d = parseFile(root, "measurement.txt")
    e_t, e_r, e_d = parseFile(root, "estimation.txt")
    g_t, g_r, g_d = parseFile(root, "groundtruth.txt")
    
    fig = plt.figure()
    
    ax_m = plt.subplot(211)
    ax_e = plt.subplot(212)
    
    
    
    if SHOW_DEGREE:
        ax_m.plot(np.array(m_t), np.array(m_d) - OFFSET, c='r', label="aruco")
        ax_m.plot(np.array(g_t), np.array(g_d), c='b', label="groundtruth")
        
        ax_e.plot(np.array(e_t), np.array(e_d) - OFFSET, c='g', label="filtered")
        ax_e.plot(np.array(g_t), np.array(g_d), c='b', label="groundtruth")
    else:
        plt.plot(np.array(m_t), np.array(m_r), c='r', label="aruco")
        plt.plot(np.array(e_t), np.array(e_r), c='g', label="filtered")
        plt.plot(np.array(g_t), np.array(g_r), c='b', label="groundtruth")

    plt.legend(loc="lower right")

    rmse_measurenment_gt = calculateRMSE(m_t, g_t, m_d, g_d)
    rmse_estimation_gt = calculateRMSE(e_t, g_t, e_d, g_d)

    print(f"rmse-aruco: {rmse_measurenment_gt} in degree")
    print(f"rmse-filtered: {rmse_estimation_gt} in degree")

    ax_m.text(np.min(g_t), np.min(g_d), f"rmse of measurement: {rmse_measurenment_gt}", fontsize=12, style="normal")
    ax_e.text(np.min(g_t), np.min(g_d) + 3, f"rmse of estimation: {rmse_estimation_gt}", fontsize=12, style="normal")
    
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root", "-r", default="/home/ubuntu/Workspace/KIT/slamdog/my_aruco_ws/src/my_aruco/eval/test")
    args = parser.parse_args()

    draw(args.root)
