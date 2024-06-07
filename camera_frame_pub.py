from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import yaml

import g2o

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class StaticFramePublisher(Node):

    def __init__(self):
        super().__init__('static_optical_frame_tf2_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'realsense_color_calib'
        t.child_frame_id = 'camera_depth_optical_frame'

        extrs_depth = g2o.Isometry3d([[0.9998, 0.0156, -0.0039, 0.0149], [-0.0156, 0.9998, -0.0040, 0.0006], [0.0038, 0.004, 0.9999, 0.0002], [0, 0, 0, 1]])
        # extrs_depth = extrs_depth.inverse()

        extrs_depth_1 = g2o.Isometry3d([[0.9998, -0.0156, -0.0039, 0.0149], [0.0156, -0.9998, -0.0387, 0.0006], [-0.0039, 0.004, 0.9999, 0.0002], [0, 0, 0, 1]])

        t.transform.translation.x = extrs_depth.t[0]
        t.transform.translation.y = extrs_depth.t[1]
        t.transform.translation.z = extrs_depth.t[2]
        
        t.transform.rotation.x = extrs_depth.orientation().x()
        t.transform.rotation.y = extrs_depth.orientation().y()
        t.transform.rotation.z = extrs_depth.orientation().z()
        t.transform.rotation.w = extrs_depth.orientation().w()

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()