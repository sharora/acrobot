import rclpy
from rclpy.node import Node

# message types for the publisher and subscriber
from std_msgs.msg import Float64MultiArray, Float64

# constants
PUBLISHER_RATE = 100 # Hz

class Controller(Node):
    """
    Controller class inherits from the node class and publishes the controls for the 
    acrobot and listens to the state of the acrobot.
    """
    def __init__(self):
        """
        Initialize the node and create the publisher and subscriber.
        """
        super().__init__('controller')
        self.state_subsription_ = self.create_subscription(Float64MultiArray, 'acrobot_state', self.subscriber_callback, 1)

        self.publisher_ = self.create_publisher(Float64, 'acrobot_control', 1)
        self.timer = self.create_timer(1/PUBLISHER_RATE, self.timer_callback)

    def subscriber_callback(self, msg):
        # printing message for debugging
        self.get_logger().info('I heard: "%s"' % msg.data)

    def timer_callback(self):
        msg = Float64()
        msg.data = 0.0
        self.publisher_.publish(msg)
    
def main():
    # creating node
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)

    # cleanup
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

    

    
