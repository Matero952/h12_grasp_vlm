#ok so for this hand service what do we wanna do?!
import rclpy
from rclpy.node import Node
from custom_ros_messages.action import Hand
from rclpy.action import ActionServer, CancelResponse

class HandControllerServer(Node):
    def __init__(self):
        super().__init__('ROS_h12_controller_hand_controller_service')
        self.hand_controller_service = ActionServer(
            self,
            Hand,
            execute_callback=None,
            cancel_callback=None,
        )
        #TODO MAKE THE CALLBACKS!!!!

