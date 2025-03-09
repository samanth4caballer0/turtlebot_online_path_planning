#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerRequest
import py_trees
import time
from mybehaviors import FollowPath, CheckObject, GetObject, LetObject, PlanPath


# TODO: Create any other required behavior like those to move the robot to a point,
#       add or check elements in the blackboard, ...

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    # Create Behaviors
    plan_path=PlanPath("plan_path")
    follow_path = FollowPath("follow_path")
    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    # let_object = LetObject("let_object")

    # create tree, define root and add behaviors
    root = py_trees.composites.Sequence(name="Sequence", memory=True)
    root.add_children([plan_path, 
                       follow_path, 
                       check_object,
                        get_object])
    behavior_tree = py_trees.trees.BehaviourTree(root=root)
    # call setup method of all tree behaviors
    behavior_tree.setup(timeout=15)

    # save tree as image
    rospack = rospkg.RosPack()
    filepath = rospack.get_path("pick_up_objects_task")
    py_trees.display.render_dot_tree(root=root, target_directory=filepath)

    # manual path because im lazy but you are not (O__O)
    plan_path.waypoints=[[3.0,-0.78],[3.0,0.7],[1.5,0.7]]

    # tick the tree
    try:
        while not rospy.is_shutdown():
            if behavior_tree.root.status!=py_trees.common.Status.SUCCESS:
                behavior_tree.tick()
                time.sleep(0.5)
            else:
                print("root returned success, tree done")
                break
        print("\n")
    except KeyboardInterrupt:
        print("")
        pass