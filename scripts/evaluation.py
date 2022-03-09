"""The evaluation process is based on the tf from franka robot.
"""

import rospy
import tf
from geometry_msgs.msg import TransformStamped


base_frame_id = "panda_link0"
tcp_frame_id = "panda_link7"


class TFListener():
    def __init__(self):
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.tcpPose = TransformStamped()
        self.tcpPose.header.frame_id = base_frame_id
        self.tcpPose.child_frame_id = tcp_frame_id
        
    def getTcpToBase(self) -> bool:
        try:
            trans, rot = self.buffer.lookup_transform(base_frame_id, tcp_frame_id, rospy.Time(0))  # ! not sure about the order
            self.tcpPose.transform.translation = trans
            self.tcpPose.transform.rotation = rot
            return True
        except:
            return False
        
    def getYaw(self):
        pass


if __name__ == "__main__":
    listener = TFListener()
    
    while not rospy.is_shutdown():
        if listener.getTcpToBase():
            print(listener.tcpPose.transform.rotation)
        
    
