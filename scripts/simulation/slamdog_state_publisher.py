#!/usr/bin/python
import math
import rospy
from gazebo_msgs.msg import ModelStates, LinkStates
from geometry_msgs.msg import Twist

from my_aruco_msg.msg import RobotState


class SlamDogStatePublisher:
    def __init__(self):
        # state_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.sign_sub = rospy.Subscriber("cmd_vel", Twist, self.signCallback)
        
        self.base_pub = rospy.Publisher("/slamdog_state", RobotState, queue_size=100)
        # left_wheel_vel_pub = rospy.Publisher("", data_class)
        # right_wheel_vel_pub = rospy.Publisher("", data_class)
        
        self.model_state = RobotState()
          
    # def callback(self, msg):
    #     name_list:list = msg.name
    #     pose_list:list = msg.pose
        
    #     slamdog_base_ind = name_list.index("slamdog::link_base")
    #     left_wheel_ind = name_list.index("slamdog::link_left_wheel")
    #     right_wheel_ind = name_list.index("slamdog::link_right_wheel")
        
    #     print(pose_list[slamdog_base_ind])
    
        self.sign = 1
        
    def signCallback(self, msg: Twist):
        if msg.linear.x > 0:
            self.sign = 1
        elif msg.linear.x < 0:
            self.sign = -1
        else:
            return
    
    def callback(self, msg):
        twist = msg.twist
                
        model_state: Twist = twist[1]
        
        
        x = model_state.linear.x
        y = model_state.linear.y
        
        w = model_state.angular.z
        
        self.model_state.linear = math.sqrt(x*x+y*y) * self.sign
        self.model_state.angular = w
        
        
        self.base_pub.publish(self.model_state)
        
if __name__ == "__main__":
    rospy.init_node("slamdog_state_publisher_node")

    state_pub = SlamDogStatePublisher()
    
    rospy.spin()