"""
It is based on 'turtlebot3_teleop' package
The difference is action when pressed enter key
It can save the current odometry and a map
"""

import sys, select
import tty, termios
import rospy
import os
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop
enter key : save current position, map
"""

e = """
Failed to communicate
"""


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def getVelMsg(lin_vel, ang_vel):
    return "current linear velocity : %s, angular velocity : %s" % (lin_vel, ang_vel)


def constrain(input, max, min):
    if input < min:
        x = min
    elif input > max:
        x = max
    else:
        x = input
    return x


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


class Odom:
    def __init__(self):
        self.sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.__cur_odom = {}
        self.__save_dir = os.getcwd() + '/target.txt'

    def getOdom(self, message):
        self.__cur_odom[0] = message.pose.pose.position.x
        self.__cur_odom[1] = message.pose.pose.position.y
        self.__cur_odom[2] = message.pose.pose.position.z
        self.__cur_odom[3] = message.pose.pose.orientation.x
        self.__cur_odom[4] = message.pose.pose.orientation.y
        self.__cur_odom[5] = message.pose.pose.orientation.z
        self.__cur_odom[6] = message.pose.pose.orientation.w

        for i in range(0, len(self.__cur_odom)):
            self.__cur_odom[i] = round(self.__cur_odom[i], 2)

    def saveFile(self):
        file = open(self.__save_dir, 'w')

        file.write("%")
        for i in range(0, len(self.__cur_odom)):
            file.write(str(self.__cur_odom[i]) + "%")
        file.close()

        self.printOdom()

    def printOdom(self):
        if self.__cur_odom is not None:
            message = """
----------------------------------------------------------------------------------------
position
x : %s  y : %s  z : %s

orientation
x : %s  y : %s  z : %s  w : %s
----------------------------------------------------------------------------------------
            """ % (self.__cur_odom[0], self.__cur_odom[1], self.__cur_odom[2], self.__cur_odom[3],
                   self.__cur_odom[4], self.__cur_odom[5], self.__cur_odom[6])

            print(message)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('custom_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    odom = Odom()

    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while 1:
            key = get_key()

            if key == 'w':
                target_linear_vel = constrain(target_linear_vel + LIN_VEL_STEP_SIZE, MAX_LIN_VEL, -MAX_LIN_VEL)
                getVelMsg(target_linear_vel, target_angular_vel)
            elif key == 'x':
                target_linear_vel = constrain(target_linear_vel - LIN_VEL_STEP_SIZE, MAX_LIN_VEL, -MAX_LIN_VEL)
                getVelMsg(target_linear_vel, target_angular_vel)
            elif key == 'a':
                target_angular_vel = constrain(target_angular_vel + ANG_VEL_STEP_SIZE, MAX_ANG_VEL, -MAX_ANG_VEL)
                getVelMsg(target_linear_vel, target_angular_vel)
            elif key == 'd':
                target_angular_vel = constrain(target_angular_vel - ANG_VEL_STEP_SIZE, MAX_ANG_VEL, -MAX_ANG_VEL)
                getVelMsg(target_linear_vel, target_angular_vel)
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                target_angular_vel = 0.0
                control_linear_vel = 0.0
                control_angular_vel = 0.0
                getVelMsg(target_linear_vel, target_angular_vel)
            else:
                if key == '\x03':  # ctrl + c
                    break
                elif key == '\x0d':
                    odom.saveFile()
                    os.system("rosrun map_server map_saver -f " + os.getcwd() + "/map")

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
