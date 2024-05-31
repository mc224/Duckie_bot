#!/usr/bin/env python3
from apriltag_detector import AprilTagger
import rospy
import os
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

HOST = os.environ['VEHICLE_NAME']
TOPIC_NAME = f'/{HOST}/camera_node/image/compressed'

class AprilTaggerNode(DTROS):
    def __init__(self, node_name, result_topic_name):
        super(AprilTaggerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.aprilTagger = AprilTagger(4000)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1)
        self.pub = rospy.Publisher(result_topic_name, CompressedImage, queue_size=1)

    def callback(self, msg):
        print(f'received message with type ${type(msg)}')

        converted_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')  # CV2 Image

        processed_img = self.aprilTagger.process_image(converted_img)

        compressed_result_img = self.bridge.cv2_to_compressed_imgmsg(processed_img)

        self.pub.publish(compressed_result_img)


if __name__ == '__main__':
    node = AprilTaggerNode(node_name='april_tagger_connection', result_topic_name="tagged_images")
    rospy.spin()
