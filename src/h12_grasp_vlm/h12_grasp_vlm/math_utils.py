import torch
import numpy as np
import cv2
import open3d as o3d
from geometry_msgs.msg import Point
from sensor_msgs.msg  import PointCloud2, PointField
from sensor_msgs_py   import point_cloud2
from std_msgs.msg import Header
import struct

def iou_3d(bbox1: o3d.geometry.AxisAlignedBoundingBox, bbox2: o3d.geometry.AxisAlignedBoundingBox) -> float:
    """
    Compute the 3D Intersection over Union (IoU) of two Open3D axis-aligned bounding boxes.

    Args:
        bbox1: An open3d.geometry.AxisAlignedBoundingBox instance.
        bbox2: An open3d.geometry.AxisAlignedBoundingBox instance.

    Returns:
        IoU value as a float in [0.0, 1.0].
    """
    # Get the min and max corner coordinates of each box
    min1 = np.array(bbox1.get_min_bound(), dtype=np.float64)
    max1 = np.array(bbox1.get_max_bound(), dtype=np.float64)
    min2 = np.array(bbox2.get_min_bound(), dtype=np.float64)
    max2 = np.array(bbox2.get_max_bound(), dtype=np.float64)

    # Compute the intersection box bounds
    inter_min = np.maximum(min1, min2)
    inter_max = np.minimum(max1, max2)

    # Compute intersection dimensions (clamp to zero if no overlap)
    inter_dims = np.clip(inter_max - inter_min, a_min=0.0, a_max=None)
    inter_vol = np.prod(inter_dims)

    # Compute volumes of each box
    vol1 = np.prod(max1 - min1)
    vol2 = np.prod(max2 - min2)

    # Compute union volume
    union_vol = vol1 + vol2 - inter_vol
    if union_vol <= 0:
        return 0.0

    return float(inter_vol / union_vol)

def pose_to_matrix(pose: np.ndarray) -> np.ndarray:
    """
    Convert a 6-vector [x, y, z, roll, pitch, yaw] (radians)
    into a 4×4 homogeneous transform.
    """
    pose = [float(p) for p in pose]  # ensure float type
    x, y, z, roll, pitch, yaw = pose

    # Rotation about X axis (roll)
    Rx = np.array([
        [1,            0,             0],
        [0,  np.cos(roll), -np.sin(roll)],
        [0,  np.sin(roll),  np.cos(roll)],
    ])

    # Rotation about Y axis (pitch)
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)],
    ])

    # Rotation about Z axis (yaw)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1],
    ])

    # Combined rotation: R = Rz @ Ry @ Rx
    R = Rz @ Ry @ Rx

    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [x, y, z]
    return T


def matrix_to_pose(T: np.ndarray) -> np.ndarray:
    """
    Convert a 4×4 homogeneous matrix back into a 6-vector
    [x, y, z, roll, pitch, yaw] with angles in radians.
    Assumes T[:3,:3] = Rz @ Ry @ Rx.
    """
    # translation
    x, y, z = T[:3, 3]

    # rotation matrix
    R = T[:3, :3]

    # recover pitch = asin(–R[2,0])
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))

    # to avoid gimbal‐lock edge cases you could test cos(pitch)≈0
    # but for most cases:
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw  = np.arctan2(R[1, 0], R[0, 0])

    return np.array([x, y, z, roll, pitch, yaw])

def quat_to_euler(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return np.array([roll, pitch, yaw])


def in_image(point: np.ndarray,
             obs_pose: np.ndarray,
             I: dict) -> bool:
    """
    Check if a 3D point in world coordinates projects into the image frame.

    Args:
        point: (3,) array giving the 3D point in world coordinates.
        obs_pose: (6,) array or list [x, y, z, roll, pitch, yaw] for camera pose in world.
        I: dict with camera intrinsics:
            - 'fx', 'fy': focal lengths
            - 'cx', 'cy': principal point
            - 'width', 'height': image size
            - 'model', 'coeffs': distortion model & parameters (ignored here if coeffs==0)

    Returns:
        True if the point projects within [0,width)×[0,height) and z_cam>0, else False.
    """
    # 1) build the camera-to-world transform, then invert to get world→camera
    T_cam2world = pose_to_matrix(obs_pose)
    T_world2cam = np.linalg.inv(T_cam2world)

    # 2) homogeneous world point → camera frame
    p_w = np.ones(4)
    p_w[:3] = point
    p_cam = T_world2cam @ p_w
    x_cam, y_cam, z_cam = p_cam[:3]

    # 3) must be in front of camera
    if z_cam <= 0:
        return False

    # 4) pinhole projection (no distortion since coeffs are zero)
    u = I['fx'] * (x_cam / z_cam) + I['cx']
    v = I['fy'] * (y_cam / z_cam) + I['cy']

    # 5) check image bounds
    in_x = (0.0 <= u) and (u < I['width'])
    in_y = (0.0 <= v) and (v < I['height'])
    return in_x and in_y

def is_obscured(
    pcd: o3d.geometry.PointCloud,
    depth_image: np.ndarray,
    cam_pose: list,
    I: dict,
    occlusion_tol: float = 1e-3,
    obscured_fraction: float = 0.1
) -> bool:
    """
    Returns True if ≥ obscured_fraction of pcd is hidden (depth_image closer).
    cam_pose must be a list (flat 16, nested 4×4, or 7-element pose).
    I: {"fx","fy","cx","cy","width","height"}.
    """
    # 1) Build world→camera matrix
    T_wc = pose_to_matrix(cam_pose)
    T_cw = np.linalg.inv(T_wc)

    # 2) Transform points into camera frame
    pts = np.asarray(pcd.points)
    pts_h = np.hstack([pts, np.ones((pts.shape[0],1))])
    pts_cam = (T_cw @ pts_h.T).T[:, :3]

    # 3) Keep only points in front
    mask_front = pts_cam[:,2] > 0
    pts_cam = pts_cam[mask_front]
    if pts_cam.size == 0:
        return False

    # 4) Project to pixel coords
    fx, fy = I["fx"], I["fy"]
    cx, cy = I["cx"], I["cy"]
    us = np.round(pts_cam[:,0]*fx/pts_cam[:,2] + cx).astype(int)
    vs = np.round(pts_cam[:,1]*fy/pts_cam[:,2] + cy).astype(int)

    H, W = I["height"], I["width"]
    valid = (us>=0)&(us<W)&(vs>=0)&(vs<H)
    us, vs, zs = us[valid], vs[valid], pts_cam[valid,2]
    if zs.size == 0:
        return False

    # 5) Compare to depth image
    depth_at = depth_image[vs, us]
    good = (depth_at>0)&np.isfinite(depth_at)
    zs, depth_at = zs[good], depth_at[good]
    if zs.size == 0:
        return False

    occluded = depth_at + occlusion_tol < zs
    frac = np.count_nonzero(occluded) / zs.size

    return frac >= obscured_fraction