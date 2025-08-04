#WHAT do we want this file to do
import rclpy
from rclpy.node import Node
from sensor_msgs.msg  import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import time
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
        print(f'{num_points=}')
        # breakpoint()
        points_xyz_rgb = np.zeros((num_points, 6), dtype=np.float32)
        for i in range(num_points):
            point_offset = i * msg.point_step
            x = struct.unpack('f', msg.data[point_offset:point_offset+4])[0]
            y = struct.unpack('f', msg.data[point_offset+4:point_offset+8])[0]
            z = struct.unpack('f', msg.data[point_offset+8:point_offset+12])[0]
            print(f'{x=}, {y=}, {z=}')
            rgb_float = struct.unpack('f', msg.data[point_offset+12:point_offset+16])[0]
            rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]
            r = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF  
            b = rgb_int & 0xFF
            print(f'{r=}, {g=}, {b=}')
            points_xyz_rgb[i] = [x, y, z, r, g, b]
            # breakpoint()
        return points_xyz_rgb

    def get_unique_bbox_points(marker: Marker):
        #the marker has repeated points bc of the way it defines 3d bboxes
        #and their edges, so this function gets the unique points from the 
        #geometry_msgs.msg.Point objects
        unique_points = set()
        for p in marker.points:
            unique_points.add((p.x, p.y, p.z))
        print(f'{unique_points=}')
        print(f'{len(unique_points)=}')
        breakpoint()
        return unique_points

    def get_best_3d_box(self, ma: MarkerArray):
        #this function gets the unique bbox points and then assembles it so that
        #we can filter point cloud points.
        print(f'{ma.markers[0]}')
        msg, _ = ma
        unique_points = self.get_unique_bbox_points(msg.markers[0])
        # breakpoint()
        
        # We know the 0th index will always have the best box?!
        #ok so the markers are organized in alternating format where it goes prob marker, bounding box
        # score_box_index = []
        # #now we get best scores
        # for idx, i  in enumerate(ma.markers):
        #     if idx % 2 != 0:
        #         # print(i)
        #         # breakpoint()
        #         text = str(i.text)
        #         # print(text)
        #         score = float(re.search(pattern = r'-?\d+\.?\d*', string=text).group(0))
        #         # idx += 1
        #         # print(score)
        #         score_box_index.append([score, idx - 1])
        
        # # print(score_box_index)
        # score_box_index = sorted(score_box_index, key=lambda x: x[0], reverse=True)
        # best_box_index = score_box_index[0][1]
        # #we sort all of our score box pairs and then get the index of the highest box
        # best_box = ma.markers[best_box_index]
        # # print(best_box)
        # # print(len(best_box.points))
        # # print(score_box_index)
        # points = set()
        # #line_list format requirees specification of start point and end point of each segment in our box, so we want to remove duplicates
        # for p in best_box.points:
        #     points.add((p.x, p.y, p.z))
        # # print(len(points))
        # # print(points)
        # sorted(points, key=lambda x : (x[0], x[1], x[2]))

        # return points
        #make sure its sorted by x asacending, then y ascending, then z ascending

    # def sort_point_cloud_points(self, point_cloud):
        #ultimately, we want to basically crop the point cloud to jsut the points inside the box,
        #so i want to do a binary search over the point lcoud points and sort them same way we do with the box points
        # print(point_cloud)
        
        # print(len(point_cloud.data))
        # print(self.parse_pointcloud2_to_xyz_rgb(point_cloud))
        # point_cloud = self.parse_pointcloud2_to_xyz_rgb(point_cloud)
        # sorted_point_cloud = sorted(point_cloud, key= lambda x: (x[0], x[1], x[2]))
    #     marker_array = MarkerArray()
    #     marker = Marker()
    #     marker.header.frame_id = "map"  # Or your camera/robot/world frame
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = "bbox"
    #     marker.id = 0
    #     marker.type = Marker.CUBE
    #     marker.action = Marker.ADD

    #     # Set pose (center of the box)
    #     marker.pose.position.x = 1.0
    #     marker.pose.position.y = 2.0
    #     marker.pose.position.z = 1.5

    #     # Optionally, add rotation (quaternion)
    #     marker.pose.orientation.w = 1.0

    #     # Set size of the bounding box
    #     marker.scale.x = 1.0  # width
    #     marker.scale.y = 0.5  # height
    #     marker.scale.z = 2.0  # depth

    #     # Set color (RGBA)
    #     marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)

    #     marker.lifetime.sec = 0  # 0 = forever

    #     marker_array.markers.append(marker)
    # #     # print(sorted_point_cloud)
    #     return marker_array
        # return sorted_point_cloud


    def crop_to_best_box(self, best_box, pc):
        #requires that best_boox and pc are already sorted in x ascending, y ascending, and z ascending.
        #I think we can first eliminate those that are outside x, and then those that are outside y, and then those that are outsize z
        def get_upper_bound(pts, which_val: str, best_val):
            assert which_val == 'x' or which_val == 'y' or which_val == 'z'
            left = 0
            right = len(pts)
            if which_val == 'x':
                idx = 0
            elif which_val == 'y':
                idx = 1
            else:
                idx = 2
            while left < right:
                mid = left + (right - left) // 2
                if pts[mid][idx] <= best_val:
                    left = mid + 1
                else:
                    right = mid
            return left
        
        def get_lower_bound(pts, which_val:str, lowest_val):
            assert which_val == 'x' or which_val == 'y' or which_val == 'z'
            left = 0
            right = len(pts)
            if which_val == 'x':
                idx = 0
            elif which_val == 'y':
                idx = 1
            else:
                idx = 2

            while left < right:
                mid = left + (right - left) // 2
                if pts[mid][idx] < lowest_val:
                    left = mid + 1
                else:
                    right = mid

            return left

        lower_x_indx = get_lower_bound(pc, 'x', list(best_box)[0][0])
        upper_x_indx = get_upper_bound(pc, 'x', list(best_box)[len(best_box) - 1][0])

        pc = pc[lower_x_indx: upper_x_indx]

        lower_y_indx = get_lower_bound(pc, 'y', list(best_box)[0][1])
        upper_y_indx = get_upper_bound(pc, 'y', list(best_box)[len(best_box) - 1][1])

        pc = pc[lower_y_indx : upper_y_indx]

        lower_z_indx = get_lower_bound(pc, 'z', list(best_box)[0][2])
        upper_z_indx = get_upper_bound(pc, 'z', list(best_box)[len(best_box) - 1][2])
    
        pc = pc[lower_z_indx : upper_y_indx]

        return pc




                

        #so we first want to cut down x points, then y points, then z points
        #so we need to get lower x bound and then upper x bound, lower y bound and then upper y bound, and then lower z bound and then upper z bound
        # while (left_x < right_x):
        #     mid = left_x + (right_x - left_x) // 2
        #     if pc[mid][0] < best_box[0].x:
        #         #so if our point cloud x is less than the min x, then we move left to mid right?
        #         left_x = mid + 1
        #     elif pc[mid][0] > best_box[0].x:
        #         #so if our pount cloud x is bigger than the min x, then we move right to mid
        #         right_x = mid - 1

        #     if pc[left_x][0] > best_box[0].x and pc[right_x][0] < best_box[len(best_box) - 1].x:
        #         break
        
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
                breakpoint()
                # print(f'{best_box=}')
                # breakpoint()
            else: 
                print("TIME OUT WAITING FOR MARKERE")
            if pc:
                # sorted_pc = test_node.sort_point_cloud_points(pc)
                pass
            else:
                print("TIME OUT WAITING FOR PC")

            if pc and marker:
                test_node.best_bbox_pub.publish(best_box)
                # cropped_to_best_box = test_node.crop_to_best_box(best_box, sorted_pc)
                # print(test_node.crop_to_best_box(best_box, sorted_pc))
                # cropped_pc_message = test_node.create_cropped_cloud_ros_object(cropped_to_best_box)
                # test_node.cropped_pc_pub.publish(cropped_pc_message)
                # raise KeyboardInterrupt
                # for i in marker.markers:


                #     print(type(i))
                #     print(i.__str__)
                #     breakpoint()
                #     print(i._points)
                #     print(len(i._points))
                #     print((i._points))
                #     # for j in i.
                #     for j in i._points:
                #         print(j.x)
                #         print(j.y)
                #         print(j.z)
                #     breakpoint()

                #     # for j in i.get_fields_and_field_types():
                #     #     print(j['points'])
                #     #     counter += 1

                # # print(f"counter = {counter}")
                # raise KeyboardInterrupt
                # # limit -= 1
                pass
            else:
                print("Timed out waiting for point cloud.")
    except KeyboardInterrupt:
        print(f"keyboard interript")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

    



#Interesting if i start example client and my movearmtopointcloud at the same time my client brekas.