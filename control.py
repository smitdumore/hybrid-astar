import sys
import rospy
import numpy as np
import math
from cv_planner import astar_planner
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

global complete


def callback(msg):
    global x_pose
    global y_pose
    global yaw
    x_pose = float(msg.pose.pose.position.x)
    y_pose = float(msg.pose.pose.position.y)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _,_, yaw = euler_from_quaternion (orientation_list)

def check_goal_tolerance(point, cur_pose):
    radius = 0.05
 #   print(cur_pose)
    if(point[0]-cur_pose[0])**2 + (point[1]-cur_pose[1])**2 <= radius**2:
        return True
    return False

i = 0

def control(cmd_pub, path):

    print("following path")
    
    global i
    
    kp_ang = 2
    kp_dist = 1.5

    bearing_ori = math.atan((path[i][1] - y_pose)/((path[i][0] - x_pose) + 1e-07))

    distance = (path[i][0]-x_pose) + (path[i][1]-y_pose)
    dist_correction = distance*kp_dist

    ang_difference = bearing_ori - (yaw)

    
    ang_correction = ang_difference*kp_ang

    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    vel_msg.angular.z = ang_correction
    

    if check_goal_tolerance(path[i], (x_pose, y_pose)):
        if i == len(path) - 1:
            print("goal reached")
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            complete = True
        i += 1

    cmd_pub.publish(vel_msg)
   

def main():

    rospy.init_node('my_node', anonymous=True)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    print("AStar planner called")
    path = astar_planner()
    path = np.asarray(path, dtype=np.float64)

    for point in path: 
        point[0] = point[0]/100.0-0.5
        point[1] = point[1]/100.0-1

    complete = False

    while (not rospy.is_shutdown() and complete == False):
        control(cmd_pub, path)


if __name__ == '__main__':
    main()