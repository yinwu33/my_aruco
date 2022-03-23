import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from my_aruco.msg import AngleStamped


class PIDController:
    def __init__(self, Kp=4, Ki=0.1, Kd=0.01, default_vel=-4, control_rate=2) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.default_vel = default_vel

        self.goal_theta = 0

        self.sample_time = 1/control_rate

        # ros initialization
        self.left_pub = rospy.Publisher(
            "/slamdog/joint_left_controller/command", Float64, queue_size=1)
        self.right_pub = rospy.Publisher(
            "/slamdog/joint_right_controller/command", Float64, queue_size=1)
        self.theta_sub = rospy.Subscriber(
            "estimation", AngleStamped, callback=self.update)

        self.left_vel = Float64()
        self.left_vel.data = self.default_vel
        self.right_vel = Float64()
        self.right_vel.data = self.default_vel

        # buffers
        self.last_time: rospy.Time = None
        self.last_error = 0.0

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.accum_time = 0.0

    def logging(self, time: rospy.Time, theta: float):
        pass

    def update(self, msg: AngleStamped):
        if self.last_time == None:
            self.last_time = msg.header.stamp
            return

        # dt = (msg.header.stamp - self.last_time).to_sec()
        # self.last_time = msg.header.stamp
        self.accum_time += (msg.header.stamp - self.last_time).to_sec()
        if self.accum_time < self.sample_time:
            return

        dt = self.accum_time

        self.last_time = msg.header.stamp
        self.accum_time = 0.0

        self.current_theta = msg.radian

        error = self.goal_theta - self.current_theta

        # P term
        self.p_term = self.Kp * error

        # I term
        self.i_term += self.Ki*(error * dt)

        # D term
        self.d_term = self.Kd * (error - self.last_error)/dt
        self.last_error = error

        # output of the PID controller
        delta_vel = (self.p_term + self.i_term + self.d_term)

        # update left and right wheel velocity
        self.left_vel.data -= delta_vel/2
        self.right_vel.data += delta_vel/2

        # publish
        self.left_pub.publish(self.left_vel)
        self.right_pub.publish(self.right_vel)

        print(f"{self.left_vel.data} {self.right_vel.data}")

        # reset output
        self.left_vel.data = self.default_vel
        self.right_vel.data = self.default_vel


if __name__ == "__main__":
    rospy.init_node("pid_controller_node")
    controller = PIDController()

    rospy.spin()
