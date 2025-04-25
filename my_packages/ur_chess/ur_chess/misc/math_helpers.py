import math
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, Point

def make_vector3(x, y, z) -> Vector3:
    vec = Vector3()
    vec.x = x
    vec.y = y
    vec.z = z
    return vec

def make_quaternion(x, y, z, w) -> Quaternion:
    quat = Quaternion()
    quat.x = x
    quat.y = y
    quat.z = z
    quat.w = w
    return quat

def make_point(x, y, z) -> Point:
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point