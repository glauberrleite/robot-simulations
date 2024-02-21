import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

class GoalPose(Node):
    def __init__(self, goal_handle):
        super().__init__('goal_pose')
        self.goal_handle = goal_handle
        
        self.goal_pub = self.create_publisher(Pose, 'goal/pose', 10)
    
    def publish_pose(self):
        # Getting pose relative to the world origin
        # pose: pose array: [x y z qx qy qz qw]
        pose = sim.getObjectPose(self.goal_handle, -1)
        
        # Creating the JointState structure
        msg = Pose()
        msg.position.x = pose[0]
        msg.position.y = pose[1]
        msg.position.z = pose[2]
        msg.orientation.x = pose[3]
        msg.orientation.y = pose[4]
        msg.orientation.z = pose[5]
        msg.orientation.w = pose[6]
        
        # Publishing message
        self.goal_pub.publish(msg)


def sysCall_init():
    sim = require('sim')

    goal_handle = sim.getObject('.')

    rclpy.init()
    self.goal = GoalPose(goal_handle)

def sysCall_sensing():
    self.goal.publish_pose()
    rclpy.spin_once(self.goal, timeout_sec=0)

def sysCall_cleanup():
    self.goal.destroy_node()
    rclpy.shutdown()
