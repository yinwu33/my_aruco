from turtle import left
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

VEL = 3


class Controller:
    def __init__(self):
        self.left_pub = rospy.Publisher(
            "/slamdog/joint_left_controller/command", Float64, queue_size=1)
        self.right_pub = rospy.Publisher(
            "/slamdog/joint_right_controller/command", Float64, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", Twist, queue_size=1, callback=self.callback)

    def callback(self, msg: Twist):
        left_vel = Float64()
        right_vel = Float64()
        left_vel.data = 0
        right_vel.data = 0
        
        if msg.linear.x == 0.5:
            left_vel.data += VEL * 2
            right_vel.data += VEL * 2
        elif msg.linear.x == -0.5:
            left_vel.data += -VEL * 2
            right_vel.data += -VEL * 2
            
        if msg.angular.z == 1:
            left_vel.data += -VEL/2
            right_vel.data += VEL/2
        if msg.angular.z == -1:
            left_vel.data += VEL/2
            right_vel.data += -VEL/2


        self.left_pub.publish(left_vel)
        self.right_pub.publish(right_vel)


if __name__ == "__main__":
    rospy.init_node("controller_node")

    controller = Controller()
    rospy.spin()
