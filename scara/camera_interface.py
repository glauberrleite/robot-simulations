import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from numpy import frombuffer, reshape, uint8

class CameraSimul(Node):
    def __init__(self, camera_handle):
        super().__init__('camera_simul')
        self.camera_handle = camera_handle

        self.publisher = self.create_publisher(Image, 'camera/image', 10)

    def publish_image(self):

        image, resolution = sim.getVisionSensorImg(self.camera_handle)

        # Creating the Image structure
        msg = Image()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = resolution[0]
        msg.width = resolution[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * msg.width

        msg.data = image
        
        # Publishing message
        self.publisher.publish(msg)

def sysCall_init():
    sim = require('sim')

    camera_handle = sim.getObject('./sensor')

    rclpy.init()
    self.camera = CameraSimul(camera_handle)

def sysCall_sensing():
    self.camera.publish_image()
    rclpy.spin_once(self.camera, timeout_sec=0)

def sysCall_cleanup():
    self.camera.destroy_node()
    rclpy.shutdown()
