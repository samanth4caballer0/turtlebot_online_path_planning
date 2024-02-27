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

    # OnlinePlanner Constructor
    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, dominion, distance_threshold):

        # ATTRIBUTES
        # List of points which define the plan. None if there is no plan
        self.path = []
        # State Validity Checker object                                                 
        self.svc = StateValidityChecker(distance_threshold)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None
        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.dominion = dominion                                        

        # CONTROLLER PARAMETERS
        # Proportional linear velocity controller gain
        self.Kv = 0.5
        # Proportional angular velocity controller gain                   
        self.Kw = 0.5
        # Maximum linear velocity control action                   
        self.v_max = 0.15
        # Maximum angular velocity control action               
        self.w_max = 0.3                

        # PUBLISHERS
        # Publisher for sending velocity commands to the robot
        self.cmd_pub = None # TODO: publisher to cmd_vel_topic
        # Publisher for visualizing the path to with rviz
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        
        # SUBSCRIBERS
        self.gridmap_sub = None # TODO: subscriber to gridmap_topic from Octomap Server  
        self.odom_sub = None # TODO: subscriber to odom_topic  
        self.move_goal_sub = None # TODO: subscriber to /move_base_simple/goal published by rviz    
        
        # TIMERS
        # Timer for velocity controller
        rospy.Timer(rospy.Duration(0.1), self.controller)
    
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])

        # TODO: Store current position (x, y, yaw) as a np.array in self.current_pose var.
        self.current_pose = ...


    # Map callback: Gets the latest occupancy map published by Octomap server and update 
    # the state validity checker
    def get_gridmap(self, gridmap):
      
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:            
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)

            # If the robot is following a path, check if it is still valid
            if len(self.path) > 0:
                # create total_path adding the current position to the rest of waypoints in the path
                total_path = [self.current_pose[0:2]] + self.path
                # TODO: check total_path validity. If total_path is not valid replan
    

    # Goal callback: Get new goal from /move_base_simple/goal topic published by rviz 
    # and computes a plan to it using self.plan() method
    def get_goal(self, goal):
        if self.svc.there_is_map:
            # TODO: Store goal (x,y) as a numpy aray in self.goal var and print it 
            self.goal = ...
            
            # Plan a new path to self.goal
            self.plan()


    # Solve plan from current position to self.goal. 
    def plan(self):
        # Invalidate previous plan if available
        self.path = []

        print("Compute new path")
        # TODO: plan a path from self.current_pose to self.goal
        self.path = compute_path(...
        
        # TODO: If planning fails, consider increasing the planning time, retry the planning a few times, etc.
        ...

        if len(self.path) == 0:
            print("Path not found!")
        else:
            print("Path found")
            # Publish plan marker to visualize in rviz
            self.publish_path()
            # remove initial waypoint in the path (current pose is already reached)
            del self.path[0]                 
        

    # This method is called every 0.1s. It computes the velocity comands in order to reach the 
    # next waypoint in the path. It also sends zero velocity commands if there is no active path.
    def controller(self, event):
        v = 0
        w = 0
        if len(self.path) > 0:
            if ... # TODO: If current waypoint is reached with some tolerance move to the next waypoint. 
                ...
                # If it was the last waypoint in the path show a message indicating it 
                ...
            else: # TODO: Compute velocities using controller function in utils_lib
                v = ...
                w = ...
        
        # Publish velocity commands
        self.__send_commnd__(v, w)
    

    # PUBLISHER HELPERS
    # Transform linear and angular velocity (v, w) into a Twist message and publish it
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)


    # Publish a path as a series of line markers
    def publish_path(self):
        if len(self.path) > 1:
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
            
            for n in self.path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)
            
            
# MAIN FUNCTION
if __name__ == '__main__':
    rospy.init_node('turtlebot_online_path_planning_node')   
    node = OnlinePlanner('/projected_map', '/odom', '/cmd_vel', np.array([-10.0, 10.0, -10.0, 10.0]), 0.2)
    
    # Run forever
    rospy.spin()
