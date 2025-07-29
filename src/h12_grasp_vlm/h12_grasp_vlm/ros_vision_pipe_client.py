import rclpy
from rclpy.node import Node
from custom_ros_messages.srv import UpdateTrackedObject, Query
import time
from sensor_msgs.msg  import PointCloud2
#THE MAJORITY OF THIS CODE WAS TAKEN FROM MAX'S VISION PIPELINE AND ALTERED SLIGHTLY
class ROS_VisionPipe_Client(Node):
    def __init__(self):
        super().__init__('ROS_VisionPipe_Query_Client_Node')
        # ExampleClient.__init__()
        self.query_client = self.create_client(Query, 'vp_query_tracked_object')
        self.update_client = self.create_client(UpdateTrackedObject, 'vp_update_tracked_object')
        #client to query vision pipeline and client to update tracked objects
        self.pc_pub = self.create_publisher(PointCloud2, '/tracked_objects/pointcloud', 1)
        while not self.query_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'{self.query_client.srv_name} service is not available.')
        while not self.update_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'{self.update_client.srv_name} service is not available.')
#OK so theres a loop going on in the vision_pipeline that generates bounding boxes at every iteration.
    def __repr__(self):
        return f'Query Client: {self.query_client}; Update Client: {self.update_client}'

    def update_track_string(self, track_string:str, request_action:str):
        #Update a track string in the bounding box generation loop.
        #'add' track string adds the track string to the loop while 'remove' track string removes it from the loop.
        assert request_action == "add" or request_action == "remove", "Input a valid request action."
        req = UpdateTrackedObject.Request()
        req.object = track_string
        req.action = request_action
        future = self.update_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def query_tracked_objects(self, track_string:str):
        #When we are generating bounding boxes for our strings, we store point clouds for each string with 
        #varying probabilities, so this method gets the highest probability point cloud for track_string.
        req = Query.Request()
        req.query = track_string
        future = self.query_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.pc_pub.publish(result.cloud)
        return result

def run_ros_vision_pipe_client(args=None):
    rclpy.init(args=args)
    ros_vision_pipe_client = ROS_VisionPipe_Client()
    # action_mapping = {1: 'add', 2: 'remove', 3: 'query'}
    try:
        while rclpy.ok():
            action = int(input("Enter an integer for the action (1: add, 2: remove, 3: query): "))
            if action not in [1, 2, 3]:
                print("Invalid action. Please enter 1, 2, or 3")
                continue
            if action == 1:
                track_string = input("Enter the track string to add: ")
                ats_out = ros_vision_pipe_client.update_track_string(track_string, 'add')
                print(f"add_track_string response: {ats_out}\n")
            elif action == 2:
                track_string = input("Enter the track string to remove: ")
                dts_out = ros_vision_pipe_client.update_track_string(track_string, 'remove')
                print(f"remove_track_string response: {dts_out}\n")
            elif action == 3:
                track_string = input("Enter the track string to query: ")
                print(f"Querying tracked objects for '{track_string}'...")
                q_out = ros_vision_pipe_client.query_tracked_objects(track_string)
                print(f"query_tracked_objects response: {q_out.message}\n")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        ros_vision_pipe_client.destroy_node()
        rclpy.shutdown()
        return 0
if __name__ == '__main__':
    import sys
    sys.exit(run_ros_vision_pipe_client())


