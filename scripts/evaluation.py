"""The evaluation process is based on the tf from franka robot.
"""

import os
import argparse
from datetime import datetime

import rospy
from geometry_msgs.msg import PointStamped

import numpy as np
from matplotlib import pyplot as plt

class Evaluator():
    def __init__(self, root: str, draw: bool = False):
        self.measurement_sub = rospy.Subscriber("measurement", PointStamped, callback=self.measurementCallback)
        self.estimation_sub = rospy.Subscriber("estimate", PointStamped, callback=self.estimationCallback)
        self.gt_sub = rospy.Subscriber("gt_yaw", PointStamped, callback=self.gtCallback)
        
        self.measurement_list = {"timestamp": [], "radian": [], "degree": []}
        self.estimation_list = {"timestamp": [], "radian": [], "degree": []}
        self.gt_list = {"timestamp": [], "radian": [], "degree": []}
        

    def measurementCallback(self, msg):
        # self.measurement_list.append(msg)
        text, time, radian, degree = self.parseData(msg)
        self.measurement_list["timestamp"].append(time)
        self.measurement_list["radian"].append(radian)
        self.fs_measurement.write(text)

    def estimationCallback(self, msg):
        text, time, radian, degree = self.parseData(msg)
        self.estimation_list["timestamp"].append(time)
        self.estimation_list["radian"].append(radian)
        self.fs_estimation.write(text)

    def gtCallback(self, msg):
        text, time, radian, degree = self.parseData(msg)
        self.gt_list["timestamp"].append(time)
        self.gt_list["radian"].append(radian)
        self.fs_groundtruth.write(text)
        
    def __enter__(self):
        self.fs_measurement = open(os.path.join(root, "measurement.txt"), "w")
        self.fs_estimation = open(os.path.join(root, "estimation.txt"), "w")
        self.fs_groundtruth = open(os.path.join(root, "groundtruth.txt"), "w")
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.fs_measurement.close()
        self.fs_estimation.close()
        self.fs_groundtruth.close()
        
    @staticmethod
    def parseData(msg: PointStamped):
        time = msg.header.stamp.to_sec()
        radian = msg.point.x
        degree = msg.point.y
        
        text = f"{time} {radian} {degree}\n"
        return text, time, radian, degree
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", "-f", default="/home/ubuntu/Workspace/KIT/slamdog/my_aruco_ws/src/my_aruco/eval")
    parser.add_argument("--prefix", default="")
    args = parser.parse_args()
    
    now = datetime.now()
    time_str = now.strftime(r"%Y-%m-%d-%H-%M-%S")
    time_str = args.prefix + "_" + time_str if args.prefix != "" else time_str
    
    root = os.path.join(folder, time_str)
    os.mkdir(root)
    
    rospy.init_node("evaluator_node")
    
    with Evaluator(root, draw=True) as evaluator:
        rospy.spin()
        
