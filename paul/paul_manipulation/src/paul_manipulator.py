#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot
# Modified by Marc-Olivier Thibault for the PAUL PMC project.

import sys
import time
import rospy
import moveit_commander
from kortex_driver.srv import *
from kortex_driver.msg import *
import moveit_msgs.msg
import geometry_msgs.msg
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane
from math import pi, radians
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler
from copy import deepcopy

class PAUL_manipulator(object):
  """PAUL_manipulator"""
  def __init__(self):

    # Initialize the node
    super(PAUL_manipulator, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('PAUL_manipulator')
    rospy.Subscriber('/arm_position_request', geometry_msgs.msg.Pose, self.request_pose_callback)

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      # self.scene = moveit_commander.PlanningSceneInterface()
      print "namespace: " + str(rospy.get_namespace())
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.planning_frame = self.arm_group.get_planning_frame()

      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

  def add_box(self, x, y, z, w, size, box_name = "box", timeout=4):
    
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.planning_frame
    box_pose.pose.orientation.w = w
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z

    self.scene.add_box(box_name, box_pose, size)

    return self.wait_for_state_update(box_name=box_name, box_is_known=True, timeout=timeout)

  def remove_box(self, box_name="box", timeout=4):
    self.scene.remove_world_object(box_name)

    return self.wait_for_state_update(box_name=box_name, box_is_attached=False, box_is_known=False, timeout=timeout)

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_cartesian_waypoints(self, waypoints, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    plans = []
    for pose in waypoints:
      arm_group.set_pose_target(pose)
      plans.append(arm_group.plan())

    #arm_group.clear_pose_targets()
    rospy.loginfo("before planning")
    #(plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.loginfo("after planning")
    

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(plans, wait=True) #arm_group.execute(plan, wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False

  def wait_for_state_update(self, box_name = "box", box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = self.scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in self.scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def box_is_in_scene(self, box_name = 'box'):
    return box_name in self.scene.get_known_object_names()

  def request_pose_callback(self, pose_msg):
    rospy.loginfo("Reaching requested Z-Pose..." + str(pose_msg))
    
    actual_pose = self.get_cartesian_pose()
    actual_pose.position.y += pose_msg.position.y
    actual_pose.position.z += pose_msg.position.z
    
    success = self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)


    rospy.loginfo("Reaching requested Pose..." + str(pose_msg))
    
    actual_pose = self.get_cartesian_pose()
    actual_pose.position.x += pose_msg.position.x
    # actual_pose = GeneratePose(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, 90.0, 0.0, 80.0)
    
    success = self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    
    if success:
      rospy.loginfo("Closing the gripper...")
      success &= self.reach_gripper_position(0.65)

    if success:
      rospy.loginfo("taking article...")
      actual_pose.position.z += 0.025
      success = self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)

    rospy.loginfo("Request is a " + str(success))

def example_send_gripper_command(value):
  # Initialize the request
  # Close the gripper
  req = SendGripperCommandRequest()
  finger = Finger()
  finger.finger_identifier = 0
  finger.value = value
  req.input.gripper.finger.append(finger)
  req.input.mode = GripperMode.GRIPPER_FORCE

  robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
  send_gripper_command_full_name = '/' + robot_name + '/base/send_gripper_command'
  send_gripper_commandCallback = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

  rospy.loginfo("Sending the gripper command...")

  # Call the service 
  try:
    send_gripper_commandCallback(req)
  except rospy.ServiceException:
    rospy.logerr("Failed to call SendGripperCommand")
    return False
  else:
    time.sleep(0.5)
    return True

def GeneratePose(x, y, z, theta, phi, psi):
  actual_pose = geometry_msgs.msg.Pose()
  actual_pose.position.x = x
  actual_pose.position.y = y
  actual_pose.position.z = z

  theta_x = radians(theta)
  theta_y = radians(phi)
  theta_z = radians(psi)

  q = quaternion_from_euler(theta_x, theta_y, theta_z)
  
  actual_pose.orientation.x = q[0]
  actual_pose.orientation.y = q[1]
  actual_pose.orientation.z = q[2]
  actual_pose.orientation.w = q[3]
  
  return actual_pose

def main():
  robot = PAUL_manipulator()
  # robot.add_box(0.0,0.0,0.0, 1.0, (0.2, 0.2, 0.0), box_name='table')
  robot.remove_box('table')
  robot.remove_box()
  # robot.add_box(0.0, -0.050, 0.10, 1.0, (0.4, 0.01, 0.2))
  # try:
  #   robot.remove_box()
  # except:
  #   pass
  # rospy.loginfo("Reaching Named Target Home...")
  # success = robot.reach_named_position("home")
  # print(success)

  print("Awaiting requests...")
  rospy.spin()


def test():
  example = PAUL_manipulator()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

#  if success:
#    rospy.loginfo("Reaching Named Target Vertical...")
#    success &= example.reach_named_position("vertical")
#    print (success)
  
#  if success:
#    rospy.loginfo("Reaching Joint Angles...")  
#    success &= example.reach_joint_angles(tolerance=0.01) #rad
#    print (success)
  
#  if success:
#    rospy.loginfo("Reaching Named Target Home...")
#    success &= example.reach_named_position("home")
#    print (success)

#  if success:
#    rospy.loginfo("Reaching Cartesian Pose...")
    
#    actual_pose = example.get_cartesian_pose()
#    actual_pose.position.z -= 0.2
#    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
#    print (success)
    
#  if example.degrees_of_freedom == 7 and success:
#    rospy.loginfo("Reach Cartesian Pose with constraints...")
#    # Get actual pose
#    actual_pose = example.get_cartesian_pose()
#    actual_pose.position.y -= 0.3
    
#    # Orientation constraint (we want the end effector to stay the same orientation)
#    constraints = moveit_msgs.msg.Constraints()
#    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
#    orientation_constraint.orientation = actual_pose.orientation
#    constraints.orientation_constraints.append(orientation_constraint)

#    # Send the goal
#    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
  
  if not example.box_is_in_scene('box'):
    box_added = example.add_box(0.467, 0.125, 0.133, 1.0, (0.4, 0.01, 0.2))
  # if not example.box_is_in_scene('table'):
  example.add_box(0.0,0.0,0.0, 1.0, (2.0, 2.0, 0.01), box_name='table')

  if success:
    rospy.loginfo("Reaching 1st Pose...")
    actual_pose = GeneratePose(0.546, 0.236, 0.132, 91.7, -1.8, 86.1)
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if success:
    rospy.loginfo("Reaching over box...")
    actual_pose = GeneratePose(0.498, -0.059, 0.132, 91.7, -2.0, 86.1)
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if example.is_gripper_present and success:

    rospy.loginfo("Opening the gripper...")
    success &= example.reach_gripper_position(0.99)
    rospy.loginfo("Closing the gripper...")
    success &= example.reach_gripper_position(0.35)
    #success &= example_send_gripper_command(1)

    #rospy.loginfo("Opening the gripper...")
    #success &= example.reach_gripper_position(0.99)

  # if success:
  #   rospy.loginfo("Reaching 2st Pose...")
  #   poses = []
  #   poses.append(GeneratePose(0.476, 0.122, 0.017, 88.8, -1.5, 50.5))
  #   poses.append(GeneratePose(0.476, 0.122, 0.196, 88.8, -1.5, 50.5))
  #   poses.append(GeneratePose(0.320, 0.108, 0.148, 88.8, -1.5, 50.5))
  #   poses.append(GeneratePose(0.320, 0.108, 0.117, 88.8, -1.5, 50.5))

  #   success = True
  #   for i in range(len(poses)):
  #     success = example.reach_cartesian_pose(pose=poses[i], tolerance=0.01, constraints=None)
  #     rospy.loginfo("pose " + str(i))
  #     if not success:
  #       break

  #   rospy.loginfo("Opening the gripper...")
  #   success &= example.reach_gripper_position(0.99)
  #   pose = GeneratePose(0.320, 0.108, 0.127, 88.8, -1.5, 50.5)
    
  #   success &= example.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
  #   print("After planning group function")
  #   print (success)

  if success:
    rospy.loginfo("Removing table box. Reaching last Poses...")
    example.remove_box('table')
    example.add_box(0.0,0.0,0.0, 1.0, (2.0, 0.5, 0.01), box_name='table')
    actual_pose = GeneratePose(0.025, 0.384, -0.163, 177.7, 0.9, 83.2)
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if example.is_gripper_present and success:

    rospy.loginfo("Opening the gripper...")
    success &= example.reach_gripper_position(0.99)
    rospy.loginfo("Closing the gripper...")
    success &= example.reach_gripper_position(0.35)

  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
