#!/usr/bin/python3

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from utils_lib.online_planning import StateValidityChecker, move_to_point, compute_path

class OnlinePlanner:
    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, dominion, distance_threshold):

        # Attributes
        self.path = []                                                  # List of points which define the plan. None if there is no plan
        self.svc = StateValidityChecker(distance_threshold)             # State Validity Checker object
        self.current_pose = None                                        # Current robot SE2 pose                           
        self.goal = None                                                # A goal is set
        self.last_map_time = rospy.Time.now()                           # Last time a map was received (to avoid map update too often)
        self.dominion = dominion                                        # Dominion in which the path planner will sample configurations

        # Parameters
        self.Kv = 0.5                   # Proportional linear velocity controller
        self.Kw = 0.5                   # Proportional angular velocity controller
        self.v_max = 0.15               # Maximum linear velocity control action
        self.w_max = 0.3                # Maximum angular velocity control action

        # Publishers
        self.cmd_pub = ...      # TODO: publisher to cmd_vel_topic    
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        
        # Subscribers
        self.gridmap_sub = ...        # TODO: subscriber to gridmap_topic  
        self.odom_sub = ...           # TODO: subscriber to odom_topic  
        self.move_goal_sub = ...      # TODO: subscriber to /move_base_simple/goal published by rviz    
        
        # Timers
        rospy.Timer(rospy.Duration(0.1), self.controller)
    
    # Odometry callback
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    # Goal callback
    def get_goal(self, goal):
        if self.svc.there_is_map :
            print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = None                                                    # to send zero velocity while planning
            self.path = self.plan()
        
    # Map callback
    def get_gridmap(self, gridmap):
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 2:            # to avoid map update too often
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)

            # If there is a path, check if it is still valid
            if self.path is not None and len(self.path) > 0:
                total_path = [self.current_pose[0:2]] + self.path
                # TODO: check total_path validity. If total_path is not valid make self.path = None and replan

    # Solve plan
    def plan(self):
        path = []
        trial = 0
        while len(path) == 0 and trial < 5:
            print("Compute new path")

            # TODO: plan a path from self.current_pose to self.goal
            path = ...

            trial += 1
        if trial == 5:
            print("Path not found!")
        else:
            print("Path found")
            self.publish_path(path)
            del path[0]                 # remove initial vertex (current pose already reached)
        return path


    # This method is called every 0.1. It checks which way point the robot has to face. Send zero velocity if there is no active path.
    def controller(self, event):
        v = 0
        w = 0
        if self.path is not None and len(self.path) > 0:

            # If current wait point reached with some tolerance move to next way point, otherwise move to current point
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < 2*self.svc.resolution:
                print("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    print("Final position reached!")
            else:
                
                # TODO: Get velocities from controller to send to robot
                v = ...
                w = ...
                
        self.__send_commnd__(v, w)
    

    # Publishers
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)
        
    def publish_path(self, path):
        if len(path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)
            

if __name__ == '__main__':
    rospy.init_node('turtlebot_online_path_planning')   
    node = OnlinePlanner('/projected_map', '/odom', '/cmd_vel', np.array([-15.0, 15.0]), 0.3)
    
    # Run forever
    rospy.spin()
