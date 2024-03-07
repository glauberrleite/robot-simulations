import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class ConveyorSimul(Node):
    def __init__(self, conveyor_handle):
        super().__init__('conveyor_simul')
        self.conveyor_handle = conveyor_handle
        
        self.subscriber = self.create_subscription(Float64, 'conveyor/target_vel', self.callback, 10)
        self.publisher = self.create_publisher(Float64, 'conveyor/vel', 10)

    def callback(self, msg):
        target_vel = msg.data
        
        sim.writeCustomTableData(self.conveyor_handle, '__ctrl__', {"vel":target_vel})

    def publish_vel(self):
        state = sim.readCustomTableData(self.conveyor_handle,'__state__')
                
        # Creating the JointState structure
        msg = Float64()
        msg.data = float(state['vel'])
        
        # Publishing message
        self.publisher.publish(msg)

def sysCall_init():
    sim = require('sim')

    conveyor_handle = sim.getObject('.')

    rclpy.init()
    self.conveyor = ConveyorSimul(conveyor_handle)

def sysCall_sensing():
    self.conveyor.publish_vel()
    rclpy.spin_once(self.conveyor, timeout_sec=0)

def sysCall_cleanup():
    self.conveyor.destroy_node()
    rclpy.shutdown()
