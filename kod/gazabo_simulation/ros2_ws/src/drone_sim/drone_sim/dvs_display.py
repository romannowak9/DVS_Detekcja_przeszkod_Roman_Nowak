import rclpy
from rclpy.node import Node
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image
from events_msgs.msg import EventArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import cv2


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

    def dvs_callback(self, msg: EventArray):
        # print("new dvs frame")
        dvs_frame = np.zeros((msg.height, msg.width)).astype(np.uint8) + 125

        for event in msg.events:
            if event.polarity:
                dvs_frame[event.y, event.x] = 0
            else:
                dvs_frame[event.y, event.x] = 255

        # Displaying in separate with cv2
        # cv2.imshow("drone_dvs", dvs_frame)
        # cv2.waitKey(1)

        frame_msg = Image()
        frame_msg.height = msg.height
        frame_msg.width = msg.width
        frame_msg.step = msg.width * 1
        frame_msg.data = dvs_frame.tobytes()
        frame_msg.pixel_format_type = 1
        self.display_pub_.publish(frame_msg)
        
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