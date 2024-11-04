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

        gz_pub_node = GzNode()
        self.display_pub_ = gz_pub_node.advertise(display_topic, Image)

        self.events_subscriber_ = self.create_subscription(EventArray, events_topic, self.dvs_callback, qos_profile)

        self.get_logger().info(self.get_name() + " has been started!")

        self.prev_frame_ts = None

    @staticmethod
    def time_msg_to_float(time_msg: Time) -> float:
        return time_msg.sec + time_msg.nanosec / 10**9

    @classmethod
    def exponantial_decay_aggregation(cls, events: List[Event], image_shape: Tuple[int], delta_t: float) -> np.ndarray:
        max = 0
        for event in events:
            if DvsNode.time_msg_to_float(event.ts) > max:
                max = DvsNode.time_msg_to_float(event.ts)  # Co to jest ??? Czas ?
                
        image = np.zeros(image_shape)
        for event in events:
            image[event.y, event.x] = (1 if event.polarity else -1) * np.exp(-np.abs(max - DvsNode.time_msg_to_float(event.ts)) / delta_t)
            #print(event[3] * np.exp(-np.abs(max - event[0]) / delta_t))

        normalizedImg = np.zeros(image_shape)
        normalizedImg = cv2.normalize(image,  normalizedImg, 0, 255, cv2.NORM_MINMAX)
        normalizedImg = normalizedImg.astype(np.uint8)

        # Displaying in separate with cv2
        # cv2.imshow("drone_dvs_exp_frame", normalizedImg)
        # cv2.waitKey(1)

        return normalizedImg
    
    @classmethod
    def event_frame_agregation(cls, events: List[Event], image_shape: Tuple[int]) -> np.ndarray:
        dvs_frame = (np.zeros(image_shape) + 125)

        for event in events:
            if event.polarity:
                dvs_frame[event.y, event.x] = 0
            else:
                dvs_frame[event.y, event.x] = 255

        # Displaying in separate with cv2
        # cv2.imshow("drone_dvs", dvs_frame)
        # cv2.waitKey(1)

        return dvs_frame.astype(np.uint8)

    def dvs_callback(self, msg: EventArray):
        # print("new dvs frame")

        if self.prev_frame_ts:
            # Event frame
            dvs_frame = DvsNode.event_frame_agregation(msg.events, (msg.height, msg.width))

            # Exponentially decaying time surface
            # dvs_frame = DvsNode.exponantial_decay_aggregation(msg.events, (msg.height, msg.width), DvsNode.time_msg_to_float(msg.header.stamp) - DvsNode.time_msg_to_float(self.prev_frame_ts))
        
            frame_msg = Image()
            frame_msg.height = msg.height
            frame_msg.width = msg.width
            frame_msg.step = msg.width * 1
            frame_msg.data = dvs_frame.tobytes()
            frame_msg.pixel_format_type = 1
            self.display_pub_.publish(frame_msg)

        self.prev_frame_ts = msg.header.stamp
        
        # cv2.imshow("drone_dvs", np.array(list(frame_msg.data)).astype(np.uint8).reshape([msg.height, msg.width]))
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = DvsNode("dvs_display_node")
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()
    print("\nDone")

if __name__=='__main__':
    main()