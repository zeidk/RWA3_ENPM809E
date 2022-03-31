from geometry_msgs.msg import Quaternion
import numpy as np
from tf.transformations import quaternion_from_euler


def euler_to_quaternion(roll, pitch, yaw):
    q = quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])
