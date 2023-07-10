import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *
import time
import numpy as np
from signal import signal, SIGINT



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
        self.get_logger().info('Searching for Odrive ...')
        self.odrive = odrive.find_any()
        self.get_logger().info('Found Odrive successfully')

        # finding zero position of both arms
        self.arm_1_base = self.odrive.axis0.encoder.count_in_cpr # absolute encoder
        self.arm_2_base = self.odrive.axis1.encoder.shadow_count # incremental encoder

        # history vars for angular velocity calculation
        self.last_theta_1 = None
        self.last_rel_theta_2 = None
        self.last_time = None

        # running motor setup sequence (calibration and finding index)
        self.armed = True
        self.index_search()
        self.odrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.set_motor_torque(0)
        self.get_logger().info('Configuration Complete')
        signal(SIGINT, self.destroy_node)


    def subscriber_callback(self, msg):
        # TODO(shreyas): perhaps clip/preprocess in some way
        self.set_motor_torque(msg.data)

    def timer_callback(self):
        # publish the state of the acrobot: [theta_1, theta_2, omega_1, omega_2]
        # theta_2 is the relative angle to theta_1, see diagram here for more details:
        # https://underactuated.mit.edu/acrobot.html#section1 
        msg = Float64MultiArray()

        # reading angles from odrive
        theta_1 = 2 * np.pi * (self.odrive.axis0.encoder.count_in_cpr - self.arm_1_base) / ENCODER_1_CPR
        theta_1 = theta_1 % (2 * np.pi) # probably don't actually need this line

        motor_angle = -2 * np.pi * (self.odrive.axis1.encoder.shadow_count - self.arm_2_base) / ENCODER_2_CPR 
        theta_2 = motor_angle * ARM_2_RATIO
        rel_theta_2 = theta_2 - theta_1
        rel_theta_2 = rel_theta_2 % (2 * np.pi)

        # computing velocities using angles history
        # omega_1 = 2 * np.pi * self.odrive.axis0.encoder.vel_estimate
        # omega_2 = - ARM_2_RATIO * 2 * np.pi * self.odrive.axis1.encoder.vel_estimate
        # rel_omega_2 = omega_2 - omega_1
        curr_time = time.time()
        if self.last_theta_1 is None or self.last_rel_theta_2 is None or self.last_time is None:
            omega_1 = 0.0
            rel_omega_2 = 0.0
        else:
            omega_1 = self.angle_diff(theta_1, self.last_theta_1)/(curr_time - self.last_time)
            rel_omega_2 = self.angle_diff(rel_theta_2, self.last_rel_theta_2)/(curr_time - self.last_time)
        self.last_theta_1 = theta_1
        self.last_rel_theta_2 = rel_theta_2
        self.last_time = curr_time

        msg.data = [theta_1, rel_theta_2, omega_1, rel_omega_2]
        self.publisher_.publish(msg)

    def angle_diff(self, angle_1, angle_2):
        # returns the signed smallest difference between two angles
        diff = angle_1 - angle_2
        return (diff + np.pi) % (2 * np.pi) - np.pi

    def index_search(self):
        '''
        Motor rotates to find index pin, syncing encoder to absolute position.
        '''
        self.odrive.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        time.sleep(0.1)
        while self.odrive.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

    def set_motor_torque(self, torque):
        if self.armed:
            self.odrive.axis1.controller.input_torque = torque
        else:
            self.odrive.axis1.controller.input_torque = 0

    def destroy_node(self, signal_recieved, frame):
        # so the pendulum safely stops on ctrl-c
        self.set_motor_torque(0)
        self.armed = False
        time.sleep(0.1) 
        self.get_logger().info('We should have set zero torque by now')
        super().destroy_node()

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

    
