#!/usr/bin/env python
from operator import ge
from re import T
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
    rospy.init_node('GoalPlanner', anonymous=True)

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
    group = moveit_commander.MoveGroupCommander(group_name)

    #We create a DisplayTrajectory publisher which is used 
    #later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("======= Printing robot state =====")
    print (robot.get_current_state())
    print ("")

    # Misc variables
    #self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
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
    moveit_commander.move_group.MoveGroupCommander.


  def detachBox(self,box_name):

  #TODO: Open the gripper and call the service that releases the box
    pass


  def attachBox(self,box_name):

  #TODO: Close the gripper and call the service that releases the box
    pass



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
      transformation = tf_buffer.lookup_transform(goal,'link_base', rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        print('Something went wrong')
    print(transformation)
    return transformation


  def main(self):
    #TODO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()
    goal = self.getGoal("pick")
    print("Our goal is:",goal.goal)
    tf = self.tf_goal(goal.goal)

    self.getGoal("place")

    rospy.signal_shutdown("Task Completed")



if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
