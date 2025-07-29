import rclpy
from rclpy.node import Node
from custom_ros_messages.srv import UpdateTrackedObject, Query
from vision_pipeline.vision_pipeline.ROS.RosVisionPipeline import ExampleClient

class ROS_VisionPipe_Client(Node):
    def __init__(self):
        Node.__init__('ROS_VisionPipe_Query_Client_Node')
        # ExampleClient.__init__()
        self.query_client = self.create_client(Query, 'vp_query_tracked_object')
        self.update_client = self.create_client(UpdateTrackedObject, 'vp_update_tracked_object')
        #client to query vision pipeline

        while not self.query_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'{self.query_client.srv_name} service is not available.')

        while not self.update_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'{self.update_client.srv_name} service is not available.')
#OK so theres a loop going on in the vision_pipeline that generates bounding boxes at every iteration.
    def __repr__(self):
        return f'Query Client: {self.query_client}; Update Client: {self.update_client}'

    def add_track_string(self, track_string):
        #Adds a string/'object' to the loop, so now the loop will start 
        #generating bounding boxes for this object
        req = UpdateTrackedObject.Request()
        req.object = track_string
        req.action = "add"
        future = self.update_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def remove_track_string(self, track_string):
        #Removes a string/'object' from the loop, so now the loop will stop
        #generating  bounding boxes for this 'object '
        req = UpdateTrackedObject.Request()
        req.object = track_string
        req.action = "remove"
        future = self.update_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def query_tracked_objects(self, track_string):
        #When we are generating bounding boxes for our strings, we store point clouds for each string with 
        #varying probabilities, so this method gets the highest probability point cloud for track_string.
        req = Query.Request()
        req.query = track_string
        future = self.query_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.pc_pub.publish(result.cloud)
        return result

