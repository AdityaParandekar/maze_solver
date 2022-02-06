#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.843515, Y + 0.6581).

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path
from math import atan2, hypot

x = 0.0
y = 0.0 
theta = 0.0
path = []
count = 0

error = 0.
last_error = 0.
integral = 0.
derivative = 0.

error_a = 0.
last_error_a = 0.
integral_a = 0.
derivative_a = 0.

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def get_path(request):
    global path
    for pt in request.poses:
        path.append((pt.pose.position.x+ 1.843515, pt.pose.position.y+ 0.6581))
    # print(path)

def calc_pid_linear(dx, dy):
    speed = Twist()
    speed.angular.z = 0 
    global integral, last_error, derivative, pub
    Kp = 0.75
    Kd = 0.50
    Ki = 0.

    error = hypot(dx, dy)
    integral = integral + error
    derivative = error - last_error
    speed.linear.x = Kp*error + Ki*integral + Kd*derivative
    last_error = error

    if speed.linear.x > 2.5:
        speed.linear.x = 2.5
    elif speed.linear.x < -2.5:
        speed.linear.x = -2.5

    pub.publish(speed)

def calc_pid_angular(dx, dy):
    global theta, integral_a, last_error_a, derivative_a, pub 
    angle_to_goal = atan2(dy, dx)
    speed = Twist()
    speed.linear.x = 0.
    Kp = 0.75
    Kd = 0.
    Ki = 0.

    error_a = angle_to_goal - theta
    integral_a = integral_a + error
    derivative_a = error_a - last_error_a
    speed.angular.z  = Kp*error_a + Ki*integral_a + Kd*derivative_a
    last_error_a = error_a

    if speed.angular.z > 3:
        speed.angular.z = 3
    elif speed.angular.z < -3:
        speed.angular.z = -3

    pub.publish(speed)

def pid_control():
    global path, x, y, theta, pub, count, bridge
    speed = Twist()
    if count ==len(path):
        return
    for pt in path:
        count = count + 1 
        goal = Point()
        goal.x = pt[0]
        goal.y = pt[1]

        inc_x = goal.x -x
        inc_y = goal.y -y
        angle_to_goal = atan2(inc_y, inc_x)

        while abs(inc_x) > 0.1 and abs(inc_y) > 0.1:
            inc_x = goal.x -x
            inc_y = goal.y -y
            angle_to_goal = atan2(inc_y, inc_x)

            if abs(angle_to_goal - theta) > 0.1:
                # rospy.logdebug("rotating")
                calc_pid_angular(inc_x, inc_y)
            else:
                # rospy.logdebug("translating")
                calc_pid_linear(inc_x, inc_y)

        # speed.linear.x = 0.
        # speed.angular.z = 0.
        # pub.publish(speed)

    speed.linear.x = 0.
    speed.angular.z = 0.
    pub.publish(speed)

rospy.init_node("not_actual_controller")

sub_odom = rospy.Subscriber("/odom", Odometry, newOdom)
sub_path = rospy.Subscriber("/path", Path, get_path)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


r = rospy.Rate(50)          # 50 Hz

if __name__ == "__main__" :
    while not rospy.is_shutdown():
        pid_control()
        r.sleep()