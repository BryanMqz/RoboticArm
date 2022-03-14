#!/usr/bin/env python
from shutil import move
import rospy
import sys
import tf_conversions
import tf2_ros
import tf
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp

class Planner():

  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)

    #Instantiate a RobotCommander object. 
    #This object is the outer-level interface to the robot:
    robot = moveit_commander.RobotCommander()
    #Instantiate a PlanningSceneInterface object. 
    #This object is an interface to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the xarm6's
    ## arm so we set ``group_name = xArm6``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the xArm:
    group_name = "xarm6" #| xarm6 |
    move_group = moveit_commander.MoveGroupCommander(group_name)
    gripper_name = "xarm_gripper"
    gripper_move = moveit_commander.MoveGroupCommander(gripper_name)

    #We create a DisplayTrajectory publisher which is used 
    #later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print ("======= Printing robot state =====")
    # print (robot.get_current_state())
    # print ("")

    # Misc variables
    #self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher


  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    
    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    pass

  def addObstacles(self):

    #TODO: Add obstables in the world

	
    #Cargo names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
    #goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]

  def goToPose(self,pose_goal):
    
    #TODO: Code used to move to a given position using move it
    self.robot.get_current_state()
    group_name = "xarm6" #| xarm6 |
    move_group = moveit_commander.MoveGroupCommander(group_name)
    # x = pose_goal.transform.translation.x
    # y = pose_goal.transform.translation.y
    # z = pose_goal.transform.translation.z

    # qx = pose_goal.transform.rotation.x
    # qy = pose_goal.transform.rotation.y
    # qz = pose_goal.transform.rotation.z
    # qw = pose_goal.transform.rotation.w

    pose_g = Pose()
    pose_g.orientation.w = pose_goal.transform.rotation.w
    pose_g.position.x = pose_goal.transform.translation.x
    pose_g.position.y = pose_goal.transform.translation.y
    pose_g.position.z = pose_goal.transform.translation.z
    pose_g.orientation.x = pose_goal.transform.rotation.x
    pose_g.orientation.y = pose_goal.transform.rotation.y
    pose_g.orientation.z = pose_goal.transform.rotation.z
    
    move_group.set_pose_reference_frame('sensor_frame')
    move_group.set_num_planning_attempts(100)
    move_group.set_planning_time(15)
    # move_group.set_end_effector_link('link_attach')
    # move_group.set_pose_target([pose_g.position.x,pose_g.position.y,pose_g.position.z,0.717037,0.697036,0,0])
    move_group.set_pose_target(pose_g)
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()


  def detachBox(self):


  #TODO: Open the gripper and call the service that releases the box
    pass
    gripper_name = "xarm_gripper"
    gripper_move = moveit_commander.MoveGroupCommander(gripper_name)
    gripper_move.set_named_target('open')
    gripper_move.go(wait=True)


  def attachBox(self):

  #TODO: Close the gripper and call the service that releases the box
    gripper_name = "xarm_gripper"
    gripper_move = moveit_commander.MoveGroupCommander(gripper_name)
    gripper_move.set_named_target('close')
    gripper_move.go(wait=True)



class myNode():
  def __init__(self):
    #TODO: Initialise ROS and create the service calls
    rospy.init_node('ch_node')
    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')


    

  def getGoal(self,action):
    print("Inside getgoal")
    getgoal = rospy.ServiceProxy('RequestGoal',RequestGoal)
    
    goal = getgoal(action)
    # print(goal)
    return goal
    #TODO: Call the service that will provide you with a suitable target for the movement


  def tf_goal(self, goal):
    #Listener del atacch frame para sacar la transformada con respecto al goal or from the sensor frame
    #Al parecer si es respecto al link base
    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    print("inside TF")
    tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0))
    tf2_ros.TransformListener(tf_buffer)

    try:
      transformation = tf_buffer.lookup_transform('sensor_frame', goal , rospy.Time(0), rospy.Duration(0.1))
      transformation2 = tf_buffer.lookup_transform('xarm_gripper_base_link','link_attach' , rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        print('Something went wrong')
    # print(transformation)
    # print(transformation2)
    transformation.transform.rotation =transformation2.transform.rotation
    return transformation


  def main(self):
    #TODO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()
    for i in range(3):
      goal = self.getGoal("pick")
      print("Our goal is:",goal.goal)
      goal_pose = self.tf_goal(goal.goal)
      self.planner.goToPose(goal_pose)

      #attach
      self.planner.attachBox()
      print('Starting place task')

      goal = self.getGoal("place")
      goal_pose = self.tf_goal(goal.goal)
      goal_pose.transform.translation.z += 0.2
      self.planner.goToPose(goal_pose)
      self.planner.detachBox()

    rospy.signal_shutdown("Task Completed")



if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
