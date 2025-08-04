#WHAT do we want this file to do
import rclpy
from rclpy.node import Node
from sensor_msgs.msg  import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
# from std_msgs.msg import ColorRGBA
import time
from geometry_msgs.msg._point import Point
# import pandas as pd

import struct
import numpy as np
from struct import pack
import regex as re

class MoveArmToPointCloud(Node):
    
    def __init__(self):
        super().__init__('MoveArmToPointCloudTest')
        #We want to set up a client to add remove and query objects
        self.marker_sub = self.create_subscription(MarkerArray, '/tracked_objects/markers/drill', self.marker_callback, 1)
        self.pc_sub = self.create_subscription(PointCloud2, '/tracked_objects/pointcloud/drill', self.pc_callback, 1)

        self.cropped_pc_pub = self.create_publisher(PointCloud2, '/tracked_objects/pointcloud/cropped/drill', 1)
        self.best_bbox_pub = self.create_publisher(MarkerArray, '/tracked_objects/markers/cropped/drill', 1)

        while self.count_publishers('/tracked_objects/pointcloud') < 1:
            self.get_logger().info('Waiting for pub')
            time.sleep(0.5)
        self.latest_pc = None
        self.latest_marker = None

    
    def pc_callback(self, msg):
        self.latest_pc = msg
        self.get_logger().info('Recieved point cloud MOVE ARM TO POINT CLOUD TEST.')

    def marker_callback(self, msg):
        self.latest_marker = msg
        self.get_logger().info('Recieved marker array')

    def get_point_cloud(self, timeout=5.0):
        start = time.time()
        while self.latest_pc is None and (time.time() - start < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_pc
    
    def get_marker(self, timeout=5.0):
        start = time.time()
        while self.latest_marker is None and (time.time() - start < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_marker
    
    def process_point_cloud(self):
        if self.latest_pc is not None:
            points = self.parse_pointcloud2_to_xyz_rgb(self.latest_pc)

        return points

    def parse_pointcloud2_to_xyz_rgb(self, msg):
        num_points = msg.width * msg.height
        # print(f'{num_points=}')
        # breakpoint()
        points_xyz_rgb = np.zeros((num_points, 6), dtype=np.float32)
        for i in range(num_points):
            point_offset = i * msg.point_step
            x = struct.unpack('f', msg.data[point_offset:point_offset+4])[0]
            y = struct.unpack('f', msg.data[point_offset+4:point_offset+8])[0]
            z = struct.unpack('f', msg.data[point_offset+8:point_offset+12])[0]
            # print(f'{x=}, {y=}, {z=}')
            rgb_float = struct.unpack('f', msg.data[point_offset+12:point_offset+16])[0]
            rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]
            r = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF  
            b = rgb_int & 0xFF
            # print(f'{r=}, {g=}, {b=}')
            points_xyz_rgb[i] = [x, y, z, r, g, b]
            # breakpoint()
        return points_xyz_rgb

    def get_unique_bbox_points(self, points: list[Point]):
        #the marker has repeated points bc of the way it defines 3d bboxes
        #and their edges, so this function gets the unique points from the 
        #geometry_msgs.msg.Point objects

        unique_points = set()
        for p in points:
            unique_points.add((p.x, p.y, p.z))
        print(f'{unique_points=}')
        print(f'{len(unique_points)=}')
        breakpoint()
        return unique_points

    def get_best_3d_box(self, ma: MarkerArray, debug=False):
        #this function gets the unique bbox points and then assembles it so that
        #we can filter point cloud points.
        all_points = ma.markers[0].points
        if debug:
            print(f'\n\n\n{ma.markers[0]=}')
            print(dir(Marker))
            print(f'\n\n\n\n{ma.markers[0].points}')
            print(len(ma.markers[0].points))
            print(type(ma.markers[0].points))
            # print(dir(Marker))
            # breakpoint()
            print(all_points)
            print(len(all_points))
            breakpoint()
        unique_points = self.get_unique_bbox_points(all_points)
        return unique_points
    
    def get_best_bbox_max_min_range(self, unique_points: set):
        #gets the minimum and maximum points of each coordinate so that we can check if pc points
        #are within the range
        unique_points = list(unique_points)
        foo_x, foo_y, foo_z = unique_points[0]
        min_x, max_x, min_y, max_y, min_z, max_z = foo_x, foo_x, foo_y, foo_y, foo_z, foo_z
        for p in unique_points:
            x, y, z = p
            if x < min_x:
                min_x = x
            elif x > max_x:
                max_x = x
            if y < min_y:
                min_y = y
            elif y > max_y:
                max_y = y
            if z < min_z:
                min_z = z
            elif z > max_z:
                max_z = z
        return {'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y, 'min_z': min_z, 'max_z': max_z}



    def crop_to_best_box(self, pc: np.ndarray, min_max_dict: dict):
        x_min, x_max, y_min, y_max, z_min, z_max = min_max_dict.items()
        # cropped_points_xyz_rgb = np.zeros((num_points, 6), dtype=np.float32)
        print(f'len points before:{len(pc)}')
        cropped_points = []
        for p in pc:
            x, y, z, _, _, _ = p
            if x > x_min and x < x_max and y > y_min and y < y_max and z > z_min and z < z_max:
                cropped_points.append(p)

        print(f'len points after: {len(cropped_points)}')
        
    def create_cropped_cloud_ros_object(self, cropped_cloud):
        def rgb_to_float(r, g, b):
            return struct.unpack('f', pack('I', (r << 16) | (g << 8) | b))[0]
        # Your list of points
        # Pack RGB values
        processed_points = [
            (x, y, z, rgb_to_float(r, g, b)) for x, y, z, r, g, b in cropped_cloud
        ]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        # Create header (you can also set the timestamp dynamically)
        header = Header()
        header.stamp.sec = 0
        header.stamp.nanosec = 0
        header.frame_id = "map"  # change to match your TF frame

        # Create PointCloud2
        cloud = point_cloud2.create_cloud(header, fields, processed_points)
        return cloud

def main():
    rclpy.init()
    test_node = MoveArmToPointCloud()    
    # counter = 0
    try:
        while True:
            # point_cloud = test_node.get_point_cloud()
            marker = test_node.get_marker()
            pc = test_node.get_point_cloud()

            if marker:
                # with open('output2.txt', 'w') as f:
                #     f.write(str(marker))
                best_box = test_node.get_best_3d_box(marker)
                best_box_max_min_dict = test_node.get_best_bbox_max_min_range(best_box)
                print(best_box_max_min_dict)
                breakpoint()
                # print(f'{best_box=}')
                # breakpoint()
            else: 
                print("TIME OUT WAITING FOR MARKERE")
            if pc:
                pc = test_node.parse_pointcloud2_to_xyz_rgb(pc)
                print(pc[0:10])
                # sorted_pc = test_node.sort_point_cloud_points(pc)
                # pc
                pass
            else:
                print("TIME OUT WAITING FOR PC")

            if pc is not None and marker is not None:
                # test_node.best_bbox_pub.publish(best_box)
                pass

                pass
            else:
                print("Timed out waiting for point cloud.")
    except KeyboardInterrupt:
        print(f"keyboard interript")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

    



#Interesting if i start example client and my movearmtopointcloud at the same time my client brekas.