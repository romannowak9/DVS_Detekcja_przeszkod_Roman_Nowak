import rclpy
from rclpy.node import Node
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image
from events_msgs.msg import EventArray, Event
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from builtin_interfaces.msg import Time

import numpy as np
import cv2
from typing import List, Tuple
from collections import deque

from modules.dvs_obstacles_detector import DvsEvent, DvsEventArray, DvsObstacleDetector


FRAME_RATE = 24
FILTER_T = 1 / 50
FILTER_K = 4
FILTER_SIZE = 5


class DvsNode(Node):
    def __init__(self, name: str):
        super().__init__(name)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        events_topic = "/dvs/events"
        display_topic = "/dvs/view"
        obstacles_view_topic = "dvs/obstacles_view"

        gz_pub_node = GzNode()
        self.display_pub_ = gz_pub_node.advertise(display_topic, Image)
        self.obst_view_pub_ = gz_pub_node.advertise(obstacles_view_topic, Image)

        self.events_subscriber_ = self.create_subscription(EventArray, events_topic, self.dvs_callback, qos_profile)

        # Creating timer to publish event frames FRAME_RATE times per second
        self.timer_period = 1 / FRAME_RATE # seconds
        self.frame_timer_ = self.create_timer(self.timer_period, self.frame_timer_callback)

        self.detector = DvsObstacleDetector(frame_rate=FRAME_RATE, filter_t=FILTER_T, filter_k=FILTER_K, filter_size=FILTER_SIZE)

        self.get_logger().info(self.get_name() + " has been started!")

    # @staticmethod
    # def time_msg_to_float(time_msg: Time) -> float:
    #     return time_msg.sec + time_msg.nanosec / 10**9

    def dvs_callback(self, msg: EventArray):
        ''' New packet of events '''
        events = []
        for event in msg.events:
            events.append(DvsEvent(event.x, event.y, event.ts.sec, event.ts.nanosec, event.polarity))

        event_array = DvsEventArray(msg.height, msg.width, events)
        self.detector.new_events(event_array)

    def frame_timer_callback(self):
        ''' Compute new frame '''
        self.detector.compute_output_frame()

        filtered_frame = self.detector.get_filtered_frame()

        # Displaying in separate with cv2
        if filtered_frame is not None:
            print("XD")
            cv2.imshow("filtered_events", filtered_frame)
            cv2.waitKey(1)

        # Publishing event frame
        # frame_msg = Image()
        # frame_msg.height = self.detector.frame_height
        # frame_msg.width = self.detector.frame_width
        # frame_msg.step = self.detector.frame_width * 1  # 1 is size of one pixel in bytes
        # frame_msg.data = filtered_frame.tobytes()
        # frame_msg.pixel_format_type = 1
        # self.display_pub_.publish(frame_msg)


def main(args=None):
    rclpy.init(args=args)

    node = DvsNode("dvs_detect_node")
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()
    print("\nDone")

if __name__=='__main__':
    main()