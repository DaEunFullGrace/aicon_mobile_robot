import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist

MAX_SCORE = 15000.0


class PoseInitialization:
    def __init__(self):
        self.is_intialized = True
        self.pub_inital_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_gather_particle = rospy.Subscriber('/particlecloud', PoseArray, self.cbGatherParticle)

        rospy.loginfo("Pose Initialization")

    def cbGatherParticle(self, poseArray):
        if not self.is_intialized:
            return

        size = len(poseArray.poses)
        score = 0.0

        for i in range(0, size):
            for j in range(i + 1, size):
                x_i = poseArray.poses[i].position.x
                x_j = poseArray.poses[j].position.x
                y_i = poseArray.poses[i].position.y
                y_j = poseArray.poses[j].position.y
                score = score + (((x_i - x_j) * (x_i - x_j) + (y_i - y_j) * (y_i - y_j)) ** 2)
        if MAX_SCORE < score < 2 * MAX_SCORE:
            twist = Twist()
            twist.angular.z = 0.8
            self.pub_twist.publish(twist)

            rospy.loginfo("on position initializing | MSE : %s > %s" % (score, MAX_SCORE))
        elif score > 2 * MAX_SCORE:
            twist = Twist()
            twist.angular.z = -0.8
            self.pub_twist.publish(twist)

            rospy.loginfo("on position initializing | MSE : %s > %s" % (score, MAX_SCORE))
        else:
            twist = Twist()
            twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            rospy.loginfo("pose initializing completed")
            rospy.signal_shutdown("pose initializing completed")


if __name__ == "__main__":
    rospy.init_node('initial_pos_pub', anonymous=True)
    pose_initialization = PoseInitialization()
    rospy.spin()
