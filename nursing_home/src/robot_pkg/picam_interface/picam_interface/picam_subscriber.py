import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

bridge = CvBridge() 

class PiCamSubscriber(Node):
    def __init__(self):
        super().__init__('picam_subscriber')
        self.picam_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        self.image = np.empty(shape=[1])
        # self.picam_subscriber  # prevent unused variable warning

    def image_callback(self, data):
        self.image = bridge.compressed_imgmsg_to_cv2(data)
        cv2.imshow('picam_img', self.image)

        key = cv2.waitKey(33)
        


def main(args=None):
    rp.init(args=args)
    
    picam_subscriber = PiCamSubscriber()

    try :
        rp.spin(picam_subscriber)
    except KeyboardInterrupt :
        picam_subscriber.get_logger().info('Stopped by Keyboard')
    finally :
        picam_subscriber.destroy_node()
        rp.shutdown()    


if __name__ == '__main__':
    main()