#!/usr/bin/env python

import sys
import rospy
import copy, math
from math import pi, dist, fabs
from moveit_commander.conversions import pose_to_list
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32MultiArray
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

ROBOT_NAME = "20robot_arm"

if ROBOT_NAME == "20robot_arm":
    GROUP_NAME_ARM = 'arm'
    GROUP_NAME_GRIPPER = 'hand'

    GRIPPER_FRAME = 'claw_base'

    FIXED_FRAME = 'base_link'

    GRIPPER_CLOSED = 0.3
    GRIPPER_OPEN = 0.0

    GRIPPER_JOINT_NAMES = ['joint6']
   
    GRIPPER_EFFORT = [1.0]

class Controller():
    def __init__(self):

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.move_group = MoveGroupCommander(GROUP_NAME_ARM)
        self.gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        
        self.move_group.set_goal_orientation_tolerance(0.00005)
        self.move_group.set_planning_time(5)
        self.move_group.set_num_planning_attempts(5) 
        self.eef = self.move_group.get_end_effector_link()

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.move_group.allow_replanning(True)
        
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        self.move_group.set_named_target("home")
        self.move_group.go()

        self.go_to_pose_goal()
        rospy.Subscriber("/opencv/coordinates",Float32MultiArray,self.callback, queue_size = 1)

        
    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0
        # pose_goal.position.y = 0.2
        # pose_goal.position.z = 0.08
        xyz = [0.12128969759822465,.24167033948878613,0.17170549194323603]

        #move_group.set_pose_target(pose_goal)
        move_group.set_position_target(xyz, "claw_base")
        move_group.set_goal_tolerance(0.01)
        #move_group.set_random_target()

        ## Now, we call the planner to compute the plan and execute it.
        print("go")
        plan = move_group.plan()
        move_group.go()
        # Calling `stop()` ensures that there is no residual movement
        #move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        print(current_pose)
        return all_close(pose_goal, current_pose, 0.01)
        

    
        
     
    	# Subscribers
    def callback(self, ros_data):
        #self.go_to_pose_goal()
        rospy.sleep(0.1)
    	
        
def all_close(goal, actual, tolerance):
        """
        Convenience method for testing if the values in two lists are within a tolerance of each other.
        For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
        between the identical orientations q and -q is calculated correctly).
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is PoseStamped:
            return all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
            # Euclidean distance
            d = dist((x1, y1, z1), (x0, y0, z0))
            # phi = angle between orientations
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

        return True

def main(args):
    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")        
       
       
 

if __name__=='__main__':
    main(sys.argv)

