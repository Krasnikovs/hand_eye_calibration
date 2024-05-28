import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

from pynput.keyboard import Key, Listener

import numpy as np
from cv_bridge import CvBridge

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time, Duration

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.msg = msg

class ImageInfoSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            CameraInfo,
            '/color/camera_info',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.msg = msg

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_1')
        self.target_frame = self.declare_parameter(
            'target_frame', 'tool0'
        ).get_parameter_value().string_value
        
    def listener(self):
        self.tf_buffer = Buffer(Duration(seconds=1))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        t_trans = Time(seconds=0)
        t = None
        target_frame = self.target_frame
        source_frame = 'base_link'
        transform_available_future = self.tf_buffer.wait_for_transform_async(
            target_frame, source_frame, t_trans)
        rclpy.spin_until_future_complete(
            self, transform_available_future, timeout_sec=1.0)
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame = target_frame,
                source_frame = source_frame,
                time = Time(),
                timeout = Duration(seconds=2)
            )  
        except TransformException as ex:
            print(f'Error:{ex}')
            raise ex
        return t


def save_npz(t, image_sub, image_data_sub):
    transform = t.transform
    vector = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    quaternion = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
    
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_sub.msg, desired_encoding='bgr8')
    
    image_resolution = list([[image_sub.msg.width], [image_sub.msg.height]])
    image_ros_topic = '/color/image_raw'
    
    np.savez(f'./photo/test_{image_sub.msg.header.stamp.sec}', vector, quaternion, image, image_resolution, image_ros_topic) 


def main(args=None):
    rclpy.init(args=args)

    pose_sub = PoseSubscriber()
    image_sub = ImageSubscriber()
    image_data_sub = ImageInfoSubscriber()

    def look(key):
        if key == Key.space:
            t = None
            t = pose_sub.listener()
            
            rclpy.spin_once(image_sub)
            rclpy.spin_once(image_data_sub)
            save_npz(t, image_sub, image_data_sub)

        if key == Key.delete:
            return False

    with Listener(on_press = look) as listener:
        listener.join()
 
    pose_sub.destroy_node()
    image_sub.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()

 