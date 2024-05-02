import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Twist


class UAVSimul(Node):
    def __init__(self, uav_handle, base_handle, target_handle):
        client_name = sim.getObjectAlias(uav_handle)

        super().__init__(f'{client_name}_node')
        self.uav_handle = uav_handle
        self.base_handle = base_handle
        self.target_handle = target_handle
        
        self.subscription = self.create_subscription(Float64MultiArray, f'{client_name}/target_position', self.callback, 10)
        self.pose_pub = self.create_publisher(Pose, f'{client_name}/pose', 10)
        self.vel_pub = self.create_publisher(Twist, f'{client_name}/vel', 10)

    def callback(self, msg):
        target_pos = msg.data
        sim.setObjectPosition(self.target_handle, -1, [target_pos[0], target_pos[1], target_pos[2]])
    
    def publish_pose(self):
        # Getting pose relative to the base
        # pose: pose array: [x y z qx qy qz qw]
        pose = sim.getObjectPose(self.base_handle, -1)
        
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
        self.pose_pub.publish(msg)

    def publish_velocity(self):
        v_linear, v_angular = sim.getObjectVelocity(self.base_handle)

        msg = Twist()

        msg.linear.x = v_linear[0]
        msg.linear.y = v_linear[1]
        msg.linear.z = v_linear[2]
        msg.angular.x = v_angular[0]
        msg.angular.y = v_angular[1]
        msg.angular.z = v_angular[2]

        self.vel_pub.publish(msg)

def sysCall_init():
    sim = require('sim')

    target_handle = sim.getObject('.')
    base_handle = sim.getObjectParent(target_handle)
    uav_handle = sim.getObjectParent(base_handle)

    rclpy.init()
    self.uav = UAVSimul(uav_handle, base_handle, target_handle)

def sysCall_sensing():
    self.uav.publish_pose()
    self.uav.publish_velocity()
    rclpy.spin_once(self.uav, timeout_sec=0)

def sysCall_cleanup():
    self.uav.destroy_node()
    rclpy.shutdown()
