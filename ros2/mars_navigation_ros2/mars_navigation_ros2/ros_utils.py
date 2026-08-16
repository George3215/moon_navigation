import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_pose_stamped(frame_id, stamp, x, y, yaw=0.0):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation = yaw_to_quaternion(yaw)
    return msg


def make_pose_with_covariance(frame_id, stamp, x, y, yaw=0.0):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation = yaw_to_quaternion(yaw)
    return msg


def path_from_points(frame_id, stamp, points):
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for i, (x, y) in enumerate(points):
        yaw = 0.0
        if i + 1 < len(points):
            nx, ny = points[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        path.poses.append(make_pose_stamped(frame_id, stamp, x, y, yaw))
    return path
