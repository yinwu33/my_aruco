import os
import argparse

import numpy as np
from matplotlib import pyplot as plt


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

    print(count)

    return sum_squere_error/count


def parseFile(path: str, filename: str):

    timestamp_list = []
    radian_list = []
    degree_list = []

    with open(os.path.join(path, filename), "r") as f:
        for line in f:
            text = line.split(" ")
            assert len(text) == 3

            timestamp_list.append(float(text[0]))
            radian_list.append(float(text[1]))
            degree_list.append(float(text[2]))

    print(
        f"Finish reading {os.path.basename(path)} with {len(timestamp_list)} data")
    return timestamp_list, radian_list, degree_list


def draw(root: str):
    m_t, m_r, m_d = parseFile(root, "measurement.txt")
    e_t, e_r, e_d = parseFile(root, "estimation.txt")
    g_t, g_r, g_d = parseFile(root, "groundtruth.txt")

    plt.plot(np.array(m_t), np.array(m_d), 'r', label="aruco")
    plt.plot(np.array(e_t), np.array(e_d), 'g', label="filtered")
    plt.plot(np.array(g_t), np.array(g_d), 'b', label="groundtruth")
    plt.legend(loc="upper left")
    # plt.savefig(os.path.join(root, "draw.png"), dpi=800)

    rmse_measurenment_gt = calculateRMSE(m_t, g_t, m_d, g_d)
    rmse_estimation_gt = calculateRMSE(e_t, g_t, e_d, g_d)
    # plt.text(1, 1, f"rmse-aruco: {rmse_measurenment_gt} in degree", fontsize=15)
    print(f"rmse-aruco: {rmse_measurenment_gt} in degree")
    print(f"rmse-filtered: {rmse_estimation_gt} in degree")

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root", "-r", default="/home/ubuntu/Workspace/KIT/slamdog/aruco_ws/src/my_aruco/eval/test")
    args = parser.parse_args()

    draw(args.root)
