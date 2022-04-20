"""The evaluation process is based on the tf from franka robot.
"""

import os
import shutil
import argparse
from datetime import datetime

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from my_aruco.msg import AngleStamped

import numpy as np
from matplotlib import pyplot as plt

topic_measurement = "measurement"
topic_prediction = "prediction"
topic_estimation = "estimation"
topic_groundtruth = "groundtruth"


class Evaluator():
    def __init__(self, root: str, draw: bool = False):
        self.measurement_sub = rospy.Subscriber(
            topic_measurement, AngleStamped, callback=self.measurementCallback)
        self.prediction_sub = rospy.Subscriber(
            topic_prediction, AngleStamped, callback=self.predictionCallback)
        self.estimation_sub = rospy.Subscriber(
            topic_estimation, AngleStamped, callback=self.estimationCallback)
        self.gt_sub = rospy.Subscriber(
            topic_groundtruth, AngleStamped, callback=self.gtCallback)

    def measurementCallback(self, msg):
        text = self.parseData(msg)
        self.fs_measurement.write(text)

    def predictionCallback(self, msg):
        text = self.parseData(msg)
        self.fs_prediction.write(text)

    def estimationCallback(self, msg):
        text = self.parseData(msg)
        self.fs_estimation.write(text)

    def gtCallback(self, msg):
        text = self.parseData(msg)

        self.fs_groundtruth.write(text)

    def __enter__(self):
        self.fs_measurement = open(os.path.join(root, "measurement.txt"), "w")
        self.fs_prediction = open(os.path.join(root, "prediction.txt"), "w")
        self.fs_estimation = open(os.path.join(root, "estimation.txt"), "w")
        self.fs_groundtruth = open(os.path.join(root, "groundtruth.txt"), "w")

    def __exit__(self, exc_type, exc_value, traceback):
        self.fs_measurement.close()
        self.fs_prediction.close()
        self.fs_estimation.close()
        self.fs_groundtruth.close()

    @staticmethod
    def parseData(msg: AngleStamped):
        time = msg.header.stamp.to_sec()
        radian = msg.radian

        text = f"{time} {radian}\n"
        return text


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--folder", "-f", default="/home/ubuntu/Workspace/KIT/slamdog/my_aruco_ws/src/my_aruco/eval")
    parser.add_argument("--prefix", default="")
    args = parser.parse_args()

    now = datetime.now()
    time_str = now.strftime(r"%Y-%m-%d-%H-%M-%S")
    time_str = args.prefix + "_" + time_str if args.prefix != "" else time_str
    time_str = "test" # ! debug
    root = os.path.join(args.folder, time_str)
    if os.path.exists(root):
        input(f"Remove folder and its content? Ctrl+C to stop {root}")
        shutil.rmtree(root)
    os.mkdir(root)

    rospy.init_node("evaluator_node")

    with Evaluator(root, draw=True) as evaluator:
        rospy.spin()
