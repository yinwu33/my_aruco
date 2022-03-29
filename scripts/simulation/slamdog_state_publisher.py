import rospy
from gazebo_msgs.msg import ModelStates, LinkStates
from geometry_msgs.msg import Twist


class SlamDogStatePublisher:
    def __init__(self):
        # state_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        
        self.base_pub = rospy.Publisher("/slamdog_state", Twist, queue_size=100)
        # left_wheel_vel_pub = rospy.Publisher("", data_class)
        # right_wheel_vel_pub = rospy.Publisher("", data_class)
        
        self.model_state = Twist()
          
    # def callback(self, msg):
    #     name_list:list = msg.name
    #     pose_list:list = msg.pose
        
    #     slamdog_base_ind = name_list.index("slamdog::link_base")
    #     left_wheel_ind = name_list.index("slamdog::link_left_wheel")
    #     right_wheel_ind = name_list.index("slamdog::link_right_wheel")
        
    #     print(pose_list[slamdog_base_ind])
    
    def callback(self, msg):
        twist = msg.twist
                
        self.model_state = twist[1]
        
        self.base_pub.publish(twist[1])
        
        
if __name__ == "__main__":
    rospy.init_node("slamdog_state_publisher_node")

    state_pub = SlamDogStatePublisher()
    
    rospy.spin()