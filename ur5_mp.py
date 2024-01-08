#!/usr/bin/python3

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from ur5_dynamics.msg import Tracker, ColorDetected
from time import sleep

tracker = Tracker()


# Define the UR5 Manipulation and Pick-and-Place class
class ur5_mp:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("ur5_mp", anonymous=False)
        rospy.loginfo("UR5 Pick-and-Place Simulation Initialized.")
        # Subscribe to object tracking data
        self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
        self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
        
        # Subscribe to color detection data
        self.color_sub = rospy.Subscriber('color_detected', ColorDetected, self.color_callback, queue_size=1)
        
        # Initialize some variables for the pick-and-place process
        self.phase = 1
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.points=[]
        self.state_change_time = rospy.Time.now()
        self.color_detected = None  # Variable to store the color detected
        
        # Log information about starting the node and initializing MoveIt!
        rospy.loginfo("Starting node ur5_mp")
        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the UR5 arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Set the joint and planning parameters
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.5)
        self.arm.set_max_velocity_scaling_factor(.5)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters
        wpose.position.x = -0.2
        wpose.position.y = -0.2
        wpose.position.z = 0.3
        self.waypoints.append(deepcopy(wpose))
        
        # Check if the target position overlaps with the initial position
        if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.x-start_pose.position.x)**2 \
            +(wpose.position.x-start_pose.position.x)**2)<0.1:
            rospy.loginfo("Warnig: target position overlaps with the initial position!")

        # Specify default (idle) joint states # Home Position 
        self.default_joint_states = self.arm.get_current_joint_values()
        self.default_joint_states[0] = -1.57691
        self.default_joint_states[1] = -1.71667
        self.default_joint_states[2] = 1.79266
        self.default_joint_states[3] = -1.67721
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        # self.arm.set_joint_value_target(self.default_joint_states)
        self.arm.go(self.default_joint_states, wait=True)
        self.arm.stop()

        self.end_joint_states = deepcopy(self.default_joint_states)
        self.end_joint_states[0] = 0.349
        self.end_joint_states[1] = -1.3705
        
        # Initialize the transition poses for placing different color objects in bins
        # Specify end states (drop Green object)
        self.transition_pose2 = deepcopy(self.default_joint_states)
        self.transition_pose2[0] = 0.15 # go to the left box
        # self.transition_pose2[4] = 1.95
        
         # Specify end states (drop RED object)
        self.transition_pose = deepcopy(self.default_joint_states)
        self.transition_pose[0] = -4.2 # go to the left box
        # self.transition_pose[4] = -1.95

        # Specify end states (drop BLUE object)
        self.transition_pose1 = deepcopy(self.default_joint_states)
        self.transition_pose1[0] = -3.4
        self.transition_pose1[2] = 2
        self.transition_pose1[4] = -1.95
                
        # # Specify end states (drop test object)
        # self.transition_pose1 = deepcopy(self.default_joint_states)
        # self.transition_pose1[0] = -1.57691 # go to the red box
        # self.transition_pose1[1] = -0.7
        # self.transition_pose1[2] = -0.2
        # # self.transition_pose[4] = 1.95

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



    def tracking_callback(self, msg):

        self.track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.error_x = msg.error_x
        self.error_y = msg.error_y
        if len(self.pointx)>9:
            self.track_flag = True
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1

        if (self.track_flag and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6):
            rospy.loginfo("Object is being tracked. Executing pick-and-place.")
            self.execute()
            self.default_pose_flag = False
        else:
            if not self.default_pose_flag:
                self.track_flag = False
                rospy.loginfo("Object lost. Executing last position.")
                self.execute()
                self.default_pose_flag = True

    def color_callback(self, msg):
        self.color_detected = msg.color_detected

    def execute(self):
        if self.track_flag:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose
            # Initialize the waypoints list
            self.waypoints= []
            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            if len(self.pointx)>8:
                if len(self.pointx)==9:
                    x_speed = np.mean(np.asarray(self.pointx[4:8]) - np.asarray(self.pointx[3:7]))
                    wpose.position.x += 2 * x_speed
                    wpose.position.z = 0.05

                else:
                    if len(self.pointx)==11: # the object is held by the gripper
                        tracker.flag2 = 1
                        self.cxy_pub.publish(tracker)

                    if len(self.pointx)<12:
                        x_speed = np.mean(np.asarray(self.pointx[4:8])-np.asarray(self.pointx[3:7]))
                        wpose.position.x += (x_speed-self.error_x*0.015/105)

                    else:
                        if tracker.flag2:
                            self.track_flag=False
                        transition_pose = deepcopy(start_pose)
                        transition_pose.position.z = 0.4 # the height distance after picking the object  

                        self.waypoints.append(deepcopy(transition_pose))

                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)

                        self.arm.set_max_acceleration_scaling_factor(.15)
                        self.arm.set_max_velocity_scaling_factor(.15)
                        if self.color_detected == "Blue":
                            rospy.loginfo("Detected Object Color: " + self.color_detected)
                            self.arm.go(self.transition_pose1, wait=True)
                        elif self.color_detected == "Green":
                            rospy.loginfo("Detected Object Color: " + self.color_detected)
                            self.arm.go(self.transition_pose2, wait=True)
                        else:
                            rospy.loginfo("Detected Object Color: " + self.color_detected) # Red
                            self.arm.go(self.transition_pose, wait=True)
                        self.arm.stop()

                        # self.arm.go(self.end_joint_states, wait=True)
                        # self.arm.stop()
                        
                        if -0.1+0.02*self.object_cnt<0.2:
                            self.object_cnt += 1

                        self.waypoints = []
                        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                        transition_pose = deepcopy(start_pose)
                        # transition_pose.position.x -= 0.1
                        # transition_pose.position.z = self.object_cnt*0.025
                        self.waypoints.append(deepcopy(transition_pose))

                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)

                        self.phase = 2
                        tracker.flag2 = 0
                        self.cxy_pub.publish(tracker)


            # Set the next waypoint to the right 0.5 meters to go after the moving brick on conveyor
            else:
                wpose.position.x -= self.error_x*0.05/105
                wpose.position.y += self.error_y*0.04/105
                wpose.position.z = 0.15
                #wpose.position.z = 0.4005

            if self.phase == 1:
                self.waypoints.append(deepcopy(wpose))

                self.pointx.append(wpose.position.x)
                self.pointy.append(wpose.position.y)

                # Set the internal state to the current state
                # self.arm.set_pose_target(wpose)

                self.arm.set_start_state_to_current_state()

                # Plan the Cartesian path connecting the waypoints
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)
                # plan = self.arm.plan()

                # If we have a complete plan, execute the trajectory
                if 1-fraction < 0.2:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    self.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                else:
                    rospy.loginfo("Path planning failed")

        else:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []
            self.pointx = []
            self.pointy = []
            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # Set the next waypoint to the right 0.5 meters
            wpose.position.x = 0.1052
            wpose.position.y = -0.4271
            wpose.position.z = 0.4005

            wpose.orientation.x = 0.4811
            wpose.orientation.y = 0.4994
            wpose.orientation.z = -0.5121
            wpose.orientation.w = 0.5069

            self.pointx.append(wpose.position.x)
            self.pointy.append(wpose.position.y)
            self.waypoints.append(deepcopy(wpose))
            # Set the internal state to the current state
            # self.arm.set_pose_target(wpose)
            
            self.arm.set_start_state_to_current_state()

            # Plan the Cartesian path connecting the waypoints
            plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)
            # plan = self.arm.plan()

            # If we have a complete plan, execute the trajectory
            if 1-fraction < 0.2:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed")


# Create an instance of the ur5_mp class
mp=ur5_mp()

# Run the ROS main loop
rospy.spin()