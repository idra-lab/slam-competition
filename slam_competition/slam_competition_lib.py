import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py
from std_msgs.msg import String

from nav_msgs.msg import Odometry, OccupancyGrid

from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField, Imu
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage

from geometry_msgs.msg import Quaternion
import tf2_py as tf2

import scipy.spatial.transform as TF


import numpy as np

import sys


def collect_imported_names():
    """
    Return a sorted list of names in this module that were imported from other modules.
    Suitable to assign to __all__ (e.g. __all__ = collect_imported_names()).
    """
    current_mod = sys.modules.get(__name__)
    imported = []
    for name, val in list(globals().items()):

        # print(name,"       ", val, "\n")
        if name.startswith('_'):
            continue
        # modules themselves
        # if inspect.ismodule(val):
        #     imported.append(name)
        #     continue
        # # objects coming from a different module
        # try:
        #     obj_mod = inspect.getmodule(val)
        # except Exception:
        #     obj_mod = None
        # if obj_mod is not None and obj_mod is not current_mod:
        #     imported.append(name)
		
        imported.append(name)
    return sorted(set(imported))

__all__ = collect_imported_names()
