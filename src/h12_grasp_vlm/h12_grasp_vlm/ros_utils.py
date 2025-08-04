import numpy as np
import cv2
import open3d as o3d
from geometry_msgs.msg import Point
from sensor_msgs.msg  import PointCloud2, PointField
from sensor_msgs_py   import point_cloud2
from std_msgs.msg import Header
import struct

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException
from tf2_ros         import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from rclpy.duration import Duration



from math_utils import quat_to_euler


def decode_compressed_depth_image(msg) -> np.ndarray:
    """
    Decodes a ROS2 compressed depth image (format: '16UC1; compressedDepth').

    Args:
        msg (CompressedImage): CompressedImage ROS message.

    Returns:
        np.ndarray: Decoded 16-bit depth image as a NumPy array.
    """
    # Ensure format is correct
    if not msg.format.lower().endswith("compresseddepth"):
        raise ValueError(f"Unsupported format: {msg.format}")

    # The first 12 bytes of the data are the depth image header
    header_size = 12
    if len(msg.data) <= header_size:
        raise ValueError("CompressedImage data too short to contain depth header")

    # Strip the custom header
    depth_header = msg.data[:header_size]
    compressed_data = msg.data[header_size:]

    # Optional: parse header (rows, cols, format, etc.) if needed
    # rows, cols, fmt, comp = struct.unpack('<II2B', depth_header[:10])

    # Decode the remaining PNG data into a 16-bit image
    np_arr = np.frombuffer(compressed_data, dtype=np.uint8)
    depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

    if depth_image is None:
        raise ValueError("cv2.imdecode failed on compressed depth image")

    if depth_image.dtype != np.uint16:
        raise TypeError(f"Expected uint16 image, got {depth_image.dtype}")

    return depth_image

def box_to_points(box):
    # min and max
    min_pt = np.array(box.min_bound)
    max_pt = np.array(box.max_bound)

    # 8 corners
    corners = [
        [min_pt[0], min_pt[1], min_pt[2]],
        [max_pt[0], min_pt[1], min_pt[2]],
        [max_pt[0], max_pt[1], min_pt[2]],
        [min_pt[0], max_pt[1], min_pt[2]],
        [min_pt[0], min_pt[1], max_pt[2]],
        [max_pt[0], min_pt[1], max_pt[2]],
        [max_pt[0], max_pt[1], max_pt[2]],
        [min_pt[0], max_pt[1], max_pt[2]]
    ]

    # 12 edges as pairs of indices into corners list
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # top face
        (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges
    ]

    # Build list of Points for LINE_LIST (pairs form lines)
    points = []
    for i, j in edges:
        p1 = Point(x=corners[i][0], y=corners[i][1], z=corners[i][2])
        p2 = Point(x=corners[j][0], y=corners[j][1], z=corners[j][2])
        points.append(p1)
        points.append(p2)
    return points


def _get_field_info(fields):
    field_info = {}
    for f in fields:
        field_info[f.name] = {
            'offset': f.offset,
            'datatype': f.datatype,
            'count': f.count,
            'size': _get_type_size(f.datatype)
        }
    return field_info

def _get_type_size(datatype):
    if datatype == PointField.INT8: return 1
    if datatype == PointField.UINT8: return 1
    if datatype == PointField.INT16: return 2
    if datatype == PointField.UINT16: return 2
    if datatype == PointField.INT32: return 4
    if datatype == PointField.UINT32: return 4
    if datatype == PointField.FLOAT32: return 4
    if datatype == PointField.FLOAT64: return 8
    return 0

# --- pcd_to_msg (remains unchanged from previous correct version) ---
def pcd_to_msg(pcd: o3d.geometry.PointCloud, frame_id: str) -> PointCloud2:
    """
    Converts an Open3D PointCloud to a ROS2 PointCloud2 message without ros2_numpy.
    Assumes pcd has points and colors.
    """
    if not pcd.has_points():
        print("Warning: Open3D PointCloud has no points to convert to ROS2 message.")
        return PointCloud2()

    points_np = np.asarray(pcd.points)
    colors_np = np.asarray(pcd.colors)

    rgb_uint8 = (colors_np * 255).astype(np.uint8)

    rgb_packed_uint32 = (
        (rgb_uint8[:, 0].astype(np.uint32) << 16) |
        (rgb_uint8[:, 1].astype(np.uint32) << 8)  |
        (rgb_uint8[:, 2].astype(np.uint32))
    )

    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('rgb', np.float32)
    ])

    pc_data_structured = np.zeros(len(points_np), dtype=dtype)
    pc_data_structured['x'] = points_np[:, 0]
    pc_data_structured['y'] = points_np[:, 1]
    pc_data_structured['z'] = points_np[:, 2]
    pc_data_structured['rgb'] = rgb_packed_uint32.view(np.float32)

    header = Header()
    header.frame_id = frame_id

    fields = []
    fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
    fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
    fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
    fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))

    point_step = pc_data_structured.dtype.itemsize
    row_step = point_step * len(points_np)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points_np)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = row_step
    msg.is_dense = True
    msg.data = pc_data_structured.tobytes()

    return msg

# --- msg_to_pcd (UPDATED to fix ValueError) ---
def msg_to_pcd(msg: PointCloud2) -> o3d.geometry.PointCloud:
    """
    Inverse of pcd_to_msg. Converts a ROS2 PointCloud2 message to an Open3D PointCloud
    by manually parsing the data field.
    Assumes the PointCloud2 message has 'x', 'y', 'z' and 'rgb' (packed float32) fields.
    """
    pcd = o3d.geometry.PointCloud()

    total_points = msg.width * msg.height
    if total_points == 0:
        print("Warning: Received empty PointCloud2 message.")
        return pcd

    field_info = _get_field_info(msg.fields)

    if not all(f in field_info for f in ['x', 'y', 'z']):
        print("Error: PointCloud2 message missing 'x', 'y', or 'z' fields.")
        return pcd

    xyz_list = []
    rgb_list = []
    has_rgb = 'rgb' in field_info and field_info['rgb']['datatype'] == PointField.FLOAT32

    endian_char = '<' if not msg.is_bigendian else '>'
    format_float32 = endian_char + 'f'

    for i in range(total_points):
        point_start_offset = i * msg.point_step

        x_offset = field_info['x']['offset']
        y_offset = field_info['y']['offset']
        z_offset = field_info['z']['offset']

        x = struct.unpack_from(format_float32, msg.data, point_start_offset + x_offset)[0]
        y = struct.unpack_from(format_float32, msg.data, point_start_offset + y_offset)[0]
        z = struct.unpack_from(format_float32, msg.data, point_start_offset + z_offset)[0]
        xyz_list.append([x, y, z])

        if has_rgb:
            rgb_offset = field_info['rgb']['offset']
            rgb_packed_float32 = struct.unpack_from(format_float32, msg.data, point_start_offset + rgb_offset)[0]

            # --- FIX FOR ValueError: Changing the dtype of a 0d array is only supported if the itemsize is unchanged ---
            # Ensure we operate on a 1D array view, even if it has one element.
            # Convert the scalar float to a 1-element numpy array before viewing.
            rgb_packed_uint32 = np.array([rgb_packed_float32], dtype=np.float32).view(np.uint32)[0]
            # .item() is then called on the single element of the 1D array, or directly use the [0] index
            # This ensures that .view() is called on an array whose itemsize doesn't change from float32 to uint32
            # because the float32 array is 1D.

            r = ((rgb_packed_uint32 >> 16) & 0x0000FF)
            g = ((rgb_packed_uint32 >> 8) & 0x0000FF)
            b = ((rgb_packed_uint32) & 0x0000FF)

            rgb_list.append([r / 255.0, g / 255.0, b / 255.0])

    pcd.points = o3d.utility.Vector3dVector(np.asarray(xyz_list))
    if has_rgb and rgb_list:
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(rgb_list))
        #print("Info: Extracted XYZ and RGB colors from PointCloud2.")
    elif has_rgb and not rgb_list:
        print("Warning: PointCloud2 message had 'rgb' field but no color data was extracted (possibly empty cloud).")
    else:
        print("Warning: PointCloud2 message does not contain a valid 'rgb' field. Only XYZ points extracted.")

    return pcd

def transform_to_matrix(tf_msg):
    """
    Convert a geometry_msgs.msg.Transform into a 4×4 numpy array.
    """
    # Translation
    tx = tf_msg.translation.x
    ty = tf_msg.translation.y
    tz = tf_msg.translation.z

    # Quaternion
    qx = tf_msg.rotation.x
    qy = tf_msg.rotation.y
    qz = tf_msg.rotation.z
    qw = tf_msg.rotation.w

    # Build rotation matrix
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    R = np.array([
        [1 - 2*(yy + zz),   2*(xy - wz),      2*(xz + wy)],
        [2*(xy + wz),       1 - 2*(xx + zz),  2*(yz - wx)],
        [2*(xz - wy),       2*(yz + wx),      1 - 2*(xx + yy)]
    ])

    # Assemble homogeneous matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T

class TFHandler:
    def __init__(self, node, cache_time: float = 120.0):
        """
        node: any rclpy.node.Node (for logging)
        cache_time: how many seconds of past transforms to buffer
        """
        self.node = node
        self._buffer = Buffer(cache_time=Duration(seconds=cache_time))
        self._listener = TransformListener(self._buffer, node)

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        time_stamp,
        timeout_sec: float = 0.1
    ):
        """
        Return a geometry_msgs.msg.Transform or None.
        time_stamp may be rclpy.time.Time or builtin_interfaces.msg.Time.
        """
        # normalize stamp to rclpy Time
        stamp = time_stamp if isinstance(time_stamp, Time) else Time.from_msg(time_stamp)

        if not self._buffer.can_transform(
            target_frame, source_frame, stamp, Duration(seconds=timeout_sec)
        ):
            self.node.get_logger().debug(
                f"TF unavailable: {source_frame} → {target_frame} @ {stamp}"
            )
            return None

        try:
            xf = self._buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=timeout_sec)
            )
            return xf.transform

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().debug(
                f"TF lookup failed {source_frame}→{target_frame}: {e}"
            )
            return None

    def lookup_pose(
        self,
        target_frame: str,
        source_frame: str,
        time_stamp,
    ) -> list[float] | None:
        """
        Return [x, y, z, roll, pitch, yaw] in target_frame, or None.
        """
        tf = self.lookup_transform(target_frame, source_frame, time_stamp)
        if tf is None:
            return None

        t = tf.translation
        q = tf.rotation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        return [t.x, t.y, t.z, roll, pitch, yaw]