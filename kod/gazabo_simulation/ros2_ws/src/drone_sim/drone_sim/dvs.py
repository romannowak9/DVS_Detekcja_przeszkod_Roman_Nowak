from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
import rclpy.time
from sensor_msgs.msg import Image

# import event_camera_msgs.msg._event_packet as event_packet
from events_msgs.msg import EventArray, Event
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# from cv_bridge import CvBridge

import time
from typing import List
import numpy as np
import cv2
# from my_config import *


# TODO: posprzątać

# Event = List[np.uint8]
# EventArray = event_packet.EventPacket
EVENT_THRESHOLD = 13
ROBOT_NAMESPACE = "x500Drone"

class CameraNode(GzNode):
    def __init__(self):
        super().__init__()
        camera_topic = "/camera"
        dvs_topic = "/dvs/events"

        self.__events_to_publish: EventArray = EventArray()
        self.__occured_new_frame: bool = False

        self.__curr_frame = None
        self.__last_frame = None

        # Create gz publisher
        # self.gz_event_publisher_ = self.advertise(dvs_topic, EventArray)
        # Create ros2 publisher
        dvs_pub_node = rclpy.create_node("dvs_pub_node")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.ros2_event_publisher_  = dvs_pub_node.create_publisher(EventArray, dvs_topic, qos_profile)
        self.__dvs_node = dvs_pub_node

        # Create subscription
        self.camera_subscriber_ = self.subscribe(GzImage, camera_topic, self.camera_cb)


        if self.camera_subscriber_:
            print("Subscribing to type {} on topic [{}]".format(GzImage, camera_topic))
        else:
            print("Error subscribing to topic [{}]".format(camera_topic))
            return
        
        print("camera node has been started!")


        # self.br = CvBridge()

    @property
    def get_events_to_pub(self) -> EventArray:
       return self.__events_to_publish
    
    def occured_new_frame(self) -> bool:
        return self.__occured_new_frame
    
    def reset_occured_new_frame_flag(self):
        self.__occured_new_frame = False

    def camera_cb(self, msg: GzImage):
        # Given data to np.ndarray
        print(len(msg.data))
        input_data = np.array(list(msg.data)).astype(np.uint8)
        current_frame_rgb = input_data.reshape([msg.height, msg.width, 3])
        current_frame_bgr = cv2.cvtColor(current_frame_rgb, cv2.COLOR_RGB2BGR)
        self.__last_frame = self.__curr_frame
        self.__curr_frame = cv2.cvtColor(current_frame_bgr, cv2.COLOR_BGR2GRAY)

        # pixel_format_type - tutaj RGB_INT8
        # msg.header
        # msg.height
        # msg.width
        # msg.encoding  - tego nie ma
        # msg.is_bigendian - tego też
        # msg.step
        # msg.data
        #current_frame = self.br.imgmsg_to_cv2(msg)

        # Original image display
        # cv2.imshow("drone_camera", current_frame_bgr)
        # cv2.waitKey(1)

        if self.__last_frame is not None:
            # Process Delta
            pos_diff = cv2.subtract(self.__curr_frame, self.__last_frame)
            neg_diff = cv2.subtract(self.__last_frame, self.__curr_frame)

            _, pos_mask = cv2.threshold(pos_diff, EVENT_THRESHOLD, 255, cv2.THRESH_BINARY)
            _, neg_mask = cv2.threshold(neg_diff, EVENT_THRESHOLD, 255, cv2.THRESH_BINARY)

            # print(self.__last_frame.shape)
            # print(pos_mask.shape)
            # print(pos_diff.shape)

            # self.__last_frame = cv2.add(self.__last_frame, cv2.bitwise_and(pos_mask, pos_diff))
            # self.__last_frame = cv2.subtract(self.__last_frame, cv2.bitwise_and(neg_mask, neg_diff))

            # Fill Events
            # własne msgs (custom msgs) na podstawie https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_msgs/msg/Event.msg i tutoriali na youtube

            events: List[Event] = []
            self.__fill_events(pos_mask, False, events)
            self.__fill_events(neg_mask, True, events)

            events_msg = EventArray()
            
            events_msg.events = events
            events_msg.width = msg.width
            events_msg.height = msg.height

            events_msg.header.frame_id = ROBOT_NAMESPACE
            # TODO: The 'stamp' field must be a sub message of type 'Time' - może wziąć czas z topica /clock?
            events_msg.header.stamp = self.__dvs_node.get_clock().now().to_msg()  # albo rclpy.time.Time().to_msg()

            self.__events_to_publish = events_msg

            #self.gz_event_publisher_.publish(events_msg)
            self.ros2_event_publisher_.publish(events_msg)
        # else:
        #     self.__last_frame = self.__curr_frame


  

        self.occured_new_frame = True

    
    def __fill_events(self, mask, polarity: bool, events: List[Event]):
        if cv2.countNonZero(mask) != 0:
            locs = cv2.findNonZero(mask)
            for loc in locs:
                event = Event()
                event.x = int(loc[0,0])
                event.y = int(loc[0,1])
                event.ts = self.__dvs_node.get_clock().now().to_msg()  # albo rclpy.time.Time().to_msg()
                event.polarity = polarity
                events.append(event)

def main():
    rclpy.init()
    camera_node = CameraNode()
    # dvs_node = rclpy.create_node("dvs_node")
    # qos_profile = QoSProfile(
    #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
    #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    #     history=QoSHistoryPolicy.KEEP_LAST,
    #     depth=10
    # )

    # dvs_topic = "/x500/dvs"
    # event_pub = dvs_node.create_publisher(EventArray, dvs_topic, qos_profile)
    



    #rclpy.spin(dvs_node)

    # wait for shutdown
    try:
      while True:
        # if camera_node.occured_new_frame:
        #     # Publish Events
        #     events_msg = camera_node.get_events_to_pub
        #     event_pub.publish(events_msg)
        #     camera_node.reset_occured_new_frame_flag()
        #     print("new dvs frame")
        time.sleep(0.001)
    except KeyboardInterrupt:
      pass

    cv2.destroyAllWindows()
    print("\nDone")

if __name__ == '__main__':
    main()