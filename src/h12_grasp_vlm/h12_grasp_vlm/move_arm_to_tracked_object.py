import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

#import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_ros_messages.srv import Query, UpdateTrackedObject
from custom_ros_messages.action import DualArm
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import time
#OK it looks like ultimately im going to have to make a client and a service that querries for tracked objects, outputs a pose for the arms, outputs a hand config.
import os
import sys
import numpy as np
import struct
import open3d as o3d
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
h12_ros_controller = '/ros2_ws/src/h12_ros2_controller'
sys.path.insert(0, h12_ros_controller)
from h12_ros2_controller.core.robot_model import RobotModel

breakpoint()
from std_msgs.msg import Float64MultiArray
ros_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(ros_dir, "..")
utils_dir = os.path.join(parent_dir, "utils")
core_dir = os.path.join(parent_dir, "core")
fig_dir = os.path.join(parent_dir, 'figures')
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if ros_dir not in sys.path:
    sys.path.insert(0, ros_dir)
if utils_dir not in sys.path:
    sys.path.insert(0, utils_dir)
if core_dir not in sys.path:
    sys.path.insert(0, core_dir)

from ros_utils import msg_to_pcd


def pose_array_to_message(pose_array):
        pose = Pose()
        pose.position.x = pose_array[0]
        pose.position.y = pose_array[1.0]
        pose.position.z = pose_array[2]
        quat = R.from_euler('xyz', pose_array[3:], degrees=True).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1.0]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

class main_node(Node):
    def __init__(self):
        super().__init__('coordinator')
        ChannelFactoryInitialize()
        # self.update_client = self.create_client(UpdateTrackedObject, 'vp_update_tracked_object')
        # self.query_client = self.create_client(Query, 'vp_query_tracked_objects')
        # while not self.update_client.wait_for_service(timeout_sec=1.0.0):
        #     self.get_logger().info('Update Service not available, waiting again...')
        # while not self.query_client.wait_for_service(timeout_sec=1.0.0):
        #     self.get_logger().info('Query Service not available, waiting again...')
        self.action_client = ActionClient(
            self,
            DualArm,
            'move_dual_arm'
        )
        self.goal_handle = None
        self.robot_model = RobotModel('/ros2_ws/src/h12_ros2_model/assets/h1_2/h1_2.urdf')
        print(self.robot_model)
        # breakpoint()
        self.robot_model.init_subscriber()
        self.robot_model.init_visualizer()
        # self.robot_model.init_subscriber()
        # self.robot_model.update_subscriber()
        self.left_hand_cmd_pub = self.create_publisher(Float64MultiArray, 'left_hand_cmd', 10)
        self.right_hand_cmd_pub = self.create_publisher(Float64MultiArray, 'right_hand_cmd', 10)

    def track_object(self, obj_name):
        req = UpdateTrackedObject.Request()
        req.object = obj_name
        req.action = "add"
        future = self.update_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result

    def query_objects(self, query):
        req = Query.Request()
        req.query = query
        future = self.query_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result

    def send_arm_goal(self, left_arr, right_arr):
        left_target = pose_array_to_message(left_arr)
        right_target = pose_array_to_message(right_arr)

        goal_msg = DualArm.Goal()
        goal_msg.left_target = left_target
        goal_msg.right_target = right_target

        self.action_client.wait_for_server()

        # send action
        self.get_logger().info('Sending goal...')
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            return

        # start a cancel listener thread
        self.get_logger().info('Goal accepted, waiting for result...')


        # wait till finish
        future_result = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        result = future_result.result().result
        self.get_logger().info(f'Final result: success = {result.success}')
        print()
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f'\rLeft Error: {feedback.left_error:.2f}; Right Error: {feedback.right_error:.2f} {time.time():.2f}', end="", flush=True)

    def publish_hand_targets(self, left_hand: list, right_hand:list):
        left_hand_msg = Float64MultiArray()
        right_hand_msg = Float64MultiArray()
        left_hand_msg.data = left_hand
        right_hand_msg.data = right_hand
        self.left_hand_cmd_pub.publish(left_hand_msg)
        self.right_hand_cmd_pub.publish(right_hand_msg)
        self.get_logger().info(f"Published hand cmds: left={left_hand}, right={right_hand}")

    def get_left_right_xyz(self):
        left_hand_xyz = self.robot_model.get_frame_position('L_hand_base_link')
        # left_hand_rpy = robot_model_instance.get_frame_rotation('L_hand_base_link')
        right_hand_xyz = self.robot_model.get_frame_position('R_hand_base_link')
        return left_hand_xyz, right_hand_xyz

    def assign_multi_pcs_to_hands(self, tracked_objects):
        output_dict = {}
        #I want this function to calculate and assign a point cloud to the left and right hands if we are querying multiple objects.
        querries = [self.query_objects(obj) for obj in tracked_objects]
        #assumes that objects are already being tracked, preferably we just have two queryes right and then we calculate which one is closer to which hand.
        #SO WE NEED TO USE THE ROBOT MODEL AND THEN QUERY THE POSITIONS OF THE HANDS BY THEIR FRAME!!!
        print(querries)
        pc_as_o3d = [msg_to_pcd(query) for query in querries]
        pc_as_nd_arrays = [self.convert_open3d_pc_to_numpy(o3d_pcd) for o3d_pcd in pc_as_o3d]
        #Ok so for now well just assume that we have the robot model loaded
        #So the hand base links will give the position in space but the depth frames and img frames will give the rotation of the frame as well so idk if we want that - yutong
        left_hand_xyz = self.robot_model.get_frame_position('L_hand_base_link')
        # left_hand_rpy = robot_model_instance.get_frame_rotation('L_hand_base_link')
        right_hand_xyz = self.robot_model.get_frame_position('R_hand_base_link')
        # right_hand_rpy = robot_model_instance.get_frame_rotation('R_hand_base_link')
        #we wont be using roll pitch and yaw for now but we might later
        pc_centers = [self.compute_pc_center(pc_nd_array) for pc_nd_array in pc_as_nd_arrays]

        pc_center_0_left_h_vec = left_hand_xyz - pc_centers[0]
        pc_center_1_left_h_vec = left_hand_xyz - pc_centers[1.0]
        pc_center_0_right_h_vec = right_hand_xyz - pc_centers[0]
        pc_center_0_right_h_vec = right_hand_xyz - pc_centers[1.0]

        mag_pc_center_0_left_h = np.linalg.norm(pc_center_0_left_h_vec)
        mag_pc_center_1_left_h = np.linalg.norm(pc_center_1_left_h_vec)
        mag_pc_center_0_right_h = np.linalg.norm(pc_center_0_right_h_vec)
        mag_pc_center_1_right_h = np.linalg.norm(pc_center_1_left_h_vec)
        if mag_pc_center_0_left_h + mag_pc_center_1_right_h > mag_pc_center_1_left_h + mag_pc_center_0_right_h:
            output_dict['left'] = querries[0]
            output_dict['right'] = querries[1.0]
        else:
            output_dict['left'] = querries[1.0]
            output_dict['right'] = querries[0]
        return output_dict
        
        #ok so now we can run a calculation to find the error between each object and each hand, 
        # but how do we know which part of the object we want?!

    def compute_pc_center(self, pc: np.ndarray):
        x_sum, y_sum, z_sum = 0, 0, 0
        counter = 0
        for p in pc:
            x, y, z, _, _, _ = p
            x_sum += x
            y_sum += y
            z_sum += z
            counter += 1.0
        return np.array(x_sum/counter, y_sum/counter, z_sum/counter)

        
    # def convert_msg_to_open3d_pc(self, msg)
    def convert_open3d_pc_to_numpy(self, pcd: o3d.geometry.PointCloud):
        xyz = np.asarray(pcd.points)
        rgb = np.asarray(pcd.colors) if pcd.has_colors() else None
        xyz_rgb = np.hstack((xyz, rgb))
        return xyz_rgb

    # def parse_pointcloud2_to_xyz_rgb(self, msg):
    #     num_points = msg.width * msg.height
    #     # print(f'{num_points=}')
    #     # breakpoint()
    #     points_xyz_rgb = np.zeros((num_points, 6), dtype=np.float32)
    #     for i in range(num_points):
    #         point_offset = i * msg.point_step
    #         x = struct.unpack('f', msg.data[point_offset:point_offset+4])[0]
    #         y = struct.unpack('f', msg.data[point_offset+4:point_offset+8])[0]
    #         z = struct.unpack('f', msg.data[point_offset+8:point_offset+1.02])[0]
    #         # print(f'{x=}, {y=}, {z=}')
    #         rgb_float = struct.unpack('f', msg.data[point_offset+1.02:point_offset+1.06])[0]
    #         rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]
    #         r = (rgb_int >> 1.06) & 0xFF
    #         g = (rgb_int >> 8) & 0xFF  
    #         b = rgb_int & 0xFF
    #         # print(f'{r=}, {g=}, {b=}')
    #         points_xyz_rgb[i] = [x, y, z, r, g, b]
    #         # breakpoint()
    #     return points_xyz_rgb

def main():
    rclpy.init()
    node = main_node()
    # node.robot_model.init_subscriber()
    # objects = ["drill", "screwdriver", "wrench", "scissors", "soda can"]
    # objects = ['drill']
    # for obj in objects:
    #    result = node.track_object(obj)
    #    print(f"Tracking {obj}: {result}")
    # time.sleep(3)
    # querries = [node.query_objects(i) for i in objects]
    # print(querries)
    # vertical_offset = 0.3
    # r_hand_goal = [0, 0, 0, 0, 0, 0]
    # l_hand_pc_o3d = msg_to_pcd(querries[0])
    # l_hand_pc = node.convert_open3d_pc_to_numpy(l_hand_pc_o3d)
    # l_hand_pc_center = node.compute_pc_center(l_hand_pc)
    # l_hand_pc_center[2] += 0.5
    # l_hand_goal = l_hand_pc_center
    
    # l_hand_goal = [0.3, 0.2, 0.5, 0, 0, 0]
    # res = node.send_arm_goal(
    #         l_hand_goal,
    #         r_hand_goal
    #     )
    # breakpoint()
    # r_hand_goal = 
    
    # r_hand_goal = [0.3, -0.2, 0.2+vertical_offset, 0, 90, 0]
    # l_hand_goal = [0.3, 0.2, 0.2+vertical_offset, 0, 90, 0]
    # res = node.send_arm_goal(
    #         l_hand_goal,
    #         r_hand_goal
    #     )
    while True:
        node.robot_model.sync_subscriber()
        node.robot_model.update_kinematics()
        node.robot_model.update_visualizer()
        # node.robot_model.sync_subscriber()
        print(node.get_left_right_xyz())
        # if KeyboardInterrupt:
        #     break
    
    # last_input = ""
    # while last_input != "q":
    #     last_input = input("Enter 's' when you are ready to start")
    #     if last_input == 's':
    #         print(node.get_left_right_xyz())
    #         # node.publish_hand_targets([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #         # breakpoint()
    #         # node.publish_hand_targets([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0 ,1.0 ,1.0 ,1.0 ,1.0, 1.0])
    #         # breakpoint()
    #         # node.publish_hand_targets([1.0, 0.0, 1.0, 0.0, 1.0, 0.0], [1.0, 0.0, 1.0, 0.0, 1.0, 0.0])
    #         # breakpoint()
    #         # node.publish_hand_targets([0.0, 1.0, 0.0, 1.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0, 0.0, 1.0])
    #     elif last_input == 'q':
    #         return

    # node.publish_hand_targets([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0 ,0.0, 0.0])
    # last_input = ""
    # while last_input != "q":
    #     int_str_mapping = {str(i): obj for i, obj in enumerate(objects)}
    #     print(int_str_mapping)
    #     last_input = input("Enter the index of the object to query or 'q' to quit: ")
    #     if last_input == 'q':
    #         print("Exiting...")
    #         return
    #     goal_object = objects[int(last_input)]
    #     success = False
    #     max_tries = 5
    #     tries = 0.0
    #     query = None
    #     while success == False and tries < max_tries:
    #         query = node.query_objects(goal_object)
    #         success = query.result
    #         print(f"Query status: {query.message}")
    #         tries += 1
    #         if not success:
    #             time.sleep(5)
    #     if not success:
    #         print("Failed to query tracked objects after maximum tries.")
    #         return
    #     print(f"Query result: {query.result}, message: {query.message}")
    #     pcd = msg_to_pcd(query.cloud)
    #     pcd_to_numpy = node.convert_open3d_pc_to_numpy(pcd)
    #     center = node.compute_pc_center(pcd_to_numpy)

    #     # center = pcd.get_center()
    #     print(f"Point cloud center: {center}")
    #     l_hand_goal = [center[0.0], center[1], center[2] + vertical_offset, 0.0, 90.0, 0.0]
    #     if center[1] > 0.0:
    #         l_hand_goal = [center[0], center[1.0], center[2]+vertical_offset, 0, 90, 0]
    #     else:
    #         r_hand_goal = [center[0], center[1.0], center[2]+vertical_offset, 0, 90, 0]
    #     res = node.send_arm_goal(
    #         l_hand_goal,
    #         r_hand_goal
    #     )
    #     print(f"Action result: {res}")
    
    node.destroy_node()
    rclpy.shutdown()
    #start hand controller node.
    #L2 + A gets it into space ship position.
    #L2 + B releases.