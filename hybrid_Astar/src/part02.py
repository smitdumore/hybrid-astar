#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from planner_class import astar_planner
import tf.transformations as tf
from tf.transformations import euler_from_quaternion as convert_ang

class PathFollower:
    def __init__(self):
        self.complete = False
        self.counter = 0
        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0
        self.go_to_goal_dist = 0.2 ** 2
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.1

        rospy.init_node('my_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.get_pose = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        print("AStar planner called")

    def pose_callback(self, pose):
        self.odom_x = float(pose.pose.pose.position.x)
        self.odom_y = float(pose.pose.pose.position.y)
        quat = pose.pose.pose.orientation
        _, _, self.odom_yaw = convert_ang([quat.x, quat.y, quat.z, quat.w])
 
    def reached(self, point):
        if (point[0] - self.odom_x) ** 2 + (point[1] - self.odom_y) ** 2 <= self.go_to_goal_dist:
            return True
        return False

    def control(self, path):
       
        bearing_ori = math.atan((path[self.counter][1] - self.odom_y) / ((path[self.counter][0] - self.odom_x)))
        ori_error = (bearing_ori - (self.odom_yaw)) * 2

        self.cmd_vel.angular.z = ori_error

        if self.reached(path[self.counter]):
            if self.counter == len(path) - 1:
                print("goal reached")
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.complete = True
            self.counter += 1

        self.cmd_pub.publish(self.cmd_vel)

    def run(self):
        
        path = astar_planner()
        path = np.asarray(path, dtype=np.float64)

        if len(path) == 0:
            print("\nno valid path found, EXITING. wait ...\n")
            path_follower.run()
            return

        for point in path:
            point[0] = point[0] / 100.0 - 0.5
            point[1] = point[1] / 100.0 - 1

        self.complete = False

        print("following path")
        while not rospy.is_shutdown() and not self.complete:
            self.control(path)


if __name__ == '__main__':
    path_follower = PathFollower()
    path_follower.run()
