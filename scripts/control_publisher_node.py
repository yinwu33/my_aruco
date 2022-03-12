from xml.etree.ElementTree import PI
import rospy
from std_msgs.msg import Float64

M_PI = 3.1415926

RANGE = 22.5
RESOLUTION = 0.1
RATE = 100

def radian(degree):
  return degree * M_PI / 180

def degree(radian):
  return radian / M_PI * 180

def getList(start, stop, step):
  l = []
  r = start
  while r < stop:
    l.append(r)
    r += step
    
  r = stop
  while r > start:
    l.append(r)
    r -= step
      
  return l

if __name__ == "__main__":
  rospy.init_node("angle_pub_node")
  pub = rospy.Publisher("/aruco_robot/tcp_controller/command", Float64, queue_size=1)
  
  rate = rospy.Rate(RATE)
  
  while not rospy.is_shutdown():
    for theta in getList(radian(-RANGE), radian(RANGE), radian(RESOLUTION)):
      msg = Float64()
      msg.data = theta
      pub.publish(msg)
      rate.sleep()