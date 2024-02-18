import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose


class DensoSimul(Node):
    def __init__(self, robot_handle, joints_handles, end_effector_handle):
        super().__init__('denso_simul')
        self.robot_handle = robot_handle
        self.joints_handles = joints_handles
        self.end_effector_handle = end_effector_handle
        
        self.subscription = self.create_subscription(Float64MultiArray, 'denso/target_positions', self.callback, 10)
        self.js_pub = self.create_publisher(JointState, 'denso/joint_states', 10)
        self.ee_pub = self.create_publisher(Pose, 'denso/pose', 10)

    def callback(self, msg):
        joint_pos = msg.data
        for i in range(6):
            sim.setJointTargetPosition(self.joints_handles[i], joint_pos[i])
    
    def publish_joints(self):
        # Filling the lists of joints variables
        joints_name = []
        joints_pos = []
        joints_vel = []
        for i in range(6):
            joints_name.append('joint'+str(i+1))
            joints_pos.append(sim.getJointPosition(self.joints_handles[i]))
            joints_vel.append(sim.getJointVelocity(self.joints_handles[i]))
        
        # Creating the JointState structure
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joints_name
        msg.position = joints_pos
        msg.velocity = joints_vel
        
        # Publishing message
        self.js_pub.publish(msg)
    
    def publish_pose(self):
        # Getting pose relative to the base
        # pose: pose array: [x y z qx qy qz qw]
        pose = sim.getObjectPose(self.end_effector_handle, self.robot_handle)
        
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
        self.ee_pub.publish(msg)


def sysCall_init():
    sim = require('sim')

    joints_handle = []
    joints_handle.append(sim.getObject('./joint1'))
    joints_handle.append(sim.getObject('./joint2'))
    joints_handle.append(sim.getObject('./joint3'))
    joints_handle.append(sim.getObject('./joint4'))
    joints_handle.append(sim.getObject('./joint5'))
    joints_handle.append(sim.getObject('./joint6'))

    end_effector_handle = sim.getObject('./end_effector')

    robot_handle = sim.getObject('.')

    rclpy.init()
    self.denso = DensoSimul(robot_handle, joints_handle, end_effector_handle)

def sysCall_sensing():
    self.denso.publish_joints()
    self.denso.publish_pose()
    rclpy.spin_once(self.denso, timeout_sec=0)

def sysCall_cleanup():
    self.denso.destroy_node()
    rclpy.shutdown()
