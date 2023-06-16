import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *


# message types for the publisher and subscriber
from std_msgs.msg import Float64MultiArray, Float64

# constants
PUBLISHER_RATE = 100 # Hz
ARM_2_RATIO = 20/32
ENCODER_1_CPR = 2**14
ENCODER_2_CPR = 8192

class Acrobot(Node):
    """
    Acrobot class inherits from the node class and publishes the state of the 
    hardware acrobot and listens to the commands to move the acrobot.
    """

    def __init__(self):
        """
        Initialize the node and create the publisher and subscriber.
        """
        super().__init__('acrobot')

        # queue size of 1 because we only want the freshest message
        self.publisher_ = self.create_publisher(Float64MultiArray, 'acrobot_state', 1)
        self.timer = self.create_timer(1/PUBLISHER_RATE, self.timer_callback)

        self.control_subscription_ = self.create_subscription(Float64, 'acrobot_control', self.subscriber_callback, 1)

        #finding odrive
        odrive = odrive.find_any()

        # finding zero position of both arms
        self.arm_1_base = odrive.axis0.encoder.count_in_cpr # absolute encoder
        self.arm_2_base = odrive.axis1.encoder.shadow_count # incremental encoder

    def subscriber_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def timer_callback(self):
        # publish the state of the acrobot
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.publisher_.publish(msg)
    
# main function
def main():
    #creating node
    rclpy.init()
    acrobot = Acrobot()
    rclpy.spin(acrobot)

    # cleanup
    acrobot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
