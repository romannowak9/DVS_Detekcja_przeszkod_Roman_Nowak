#!/home/roman/anaconda3/envs/v2e/bin/python
import rclpy
from rclpy.node import Node
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import cv2
import time

from modules.dvs_obstacles_detector import DvsObstacleDetector

from v2ecore.emulator import EventEmulator
import torch


# TODO: Sróbować z innymi topicami - ale to raczej nic nie da

# Parametry kamery
# distortion {0 ,0 ,0 ,0 ,0}
# intrinsics {207.89351463317871 ,0 ,120 ,0 ,207.89351463317871 ,90 ,0 ,0, 1}
# projection {207.89351463317871, 0, 120, 0, 0, 207.89351463317871, 90, 0, 0, 0, 1, 0}

# topic'i:
# /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu, orientation, f = ~124Hz, Najlpesze do rotacji
# /world/default/dynamic_pose/info, name="x500_0", position, orientation, f = ~42Hz
# Oba można brać z /fmu/out/vehicle_odometry - f = ~61Hz, position i q, Najlepsze do pozycji albo do obu na raz - chyba x i y zamienione względem tego powyżej


# Local parameters
EVENTS_FRAME_RATE = 24

# Detector parameters
FRAME_EVENTS_N = 1500
UNFILTERED_FRAMES = True
SHOW_OBSTACLES = True

# Detector filter parameters
FILTER_T = 1/60
FILTER_K = 3
FILTER_SIZE = 3

# Detector RHT parameters
# SAMPLES = 1500
# BINS = 20
# MIN_VOTES = 3
# MIN_PLANE_POINTS = 40
# NUM_CANDIDATES = 5
# MIN_POINT_DIST = 0.5

# Detector obstacles detection parameters
DBSCAN_EPS = 5 # dla statycznej 7
DBSCAN_MIN_SAMPLES = 30

# Gazebo parameters
REAL_TIME_FACTOR = 0.2
CAMERA_FRAME_RATE = 100


class DvsNode(GzNode):
    def __init__(self):
        super().__init__()
        # qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10
        # )

        self.__ros2_node = Node("dvs_detect_node")

        self.__curr_frame = None
        self.__curr_frame_rgb = None
        self.__start_time = self.__ros2_node.get_clock().now().nanoseconds  # nanosec
        self.__sim_time = None  # nanosec
        self.__gz_sim_time = None
        self.__new_events = False
        self.__detector = None
        # self.__emulator = EventEmulator(
        #         pos_thres=0.2,
        #         neg_thres=0.2,
        #         sigma_thres=0.03,
        #         cutoff_hz=200,
        #         leak_rate_hz=1,
        #         shot_noise_rate_hz=10,
        #         device="cuda")
        
        # TODO: Tu może warto zrobić tak, że ograniczamy się do np. 30 FPS i polegamy na interpolacji klatek - tylko trzeba sprawdzić ile interpoluje
        self.__emulator = EventEmulator(shot_noise_rate_hz=0.35, leak_rate_hz=0.2)
        
        camera_topic = "/camera"
        # events_topic = "/dvs/events"
        display_topic = "/dvs/view"
        obstacles_view_topic = "dvs/obstacles_view"
        position_topic = "/world/default/dynamic_pose/info"

        self.__display_pub_ = self.advertise(display_topic, GzImage)
        self.__obst_view_pub_ = self.advertise(obstacles_view_topic, GzImage)

        # Subscribing gazebo camera
        self.__camera_subscriber_ = self.subscribe(GzImage, camera_topic, self.camera_cb)
        # self.__events_subscriber_ = self.subscribe(GzImage, camera_topic, self.events_cb)
        self.__clock_subscriber_ = self.subscribe(Clock, "/clock", self.clock_cb)
        self.__pose_subscriber_ = self.subscribe(Pose_V, position_topic, self.pose_cb)

        # Creating timer to publish event frames FRAME_RATE times per second
        self.timer_period = 1 / EVENTS_FRAME_RATE # seconds
        self.__frame_timer_ = self.__ros2_node.create_timer(self.timer_period, self.frame_timer_callback)
        self.__events_timer_ = self.__ros2_node.create_timer(1 / CAMERA_FRAME_RATE * REAL_TIME_FACTOR, self.events_cb)

        self.__events = None

        self.__ros2_node.get_logger().info(self.__ros2_node.get_name() + " has been started!")
        rclpy.spin(self.__ros2_node)

    def pose_cb(self, msg: Pose_V):
        if self.__detector is not None:
            for single_pose in msg.pose:
                if single_pose.name == "x500_0":
                    position = [single_pose.position.x, single_pose.position.y, single_pose.position.z]
                    orientation = [single_pose.orientation.x, single_pose.orientation.y, single_pose.orientation.z, single_pose.orientation.w]
                    
                    self.__detector.update_camera_pos(position, orientation)
                    # print(position)
                    # print(orientation)
                    break

    @staticmethod
    def gz_time_msg_to_float(gz_time) -> float:
        return gz_time.sec + gz_time.nsec / 10**9

    def clock_cb(self, msg: Clock):
        self.__gz_sim_time = msg.sim

    def camera_cb(self, msg: GzImage):
        ''' New frame '''
        if self.__detector is None:
            self.__detector = DvsObstacleDetector(frame_shape=(msg.height, msg.width),
                                                  n_frame_events=FRAME_EVENTS_N,
                                                  filter_t=FILTER_T, filter_k=FILTER_K,
                                                  filter_size=FILTER_SIZE,
                                                #   rht_samples=SAMPLES,
                                                #   rht_bins=BINS,
                                                #   rht_min_votes=MIN_VOTES,
                                                #   rht_min_plane_points=MIN_PLANE_POINTS,
                                                #   rht_min_point_dist=MIN_POINT_DIST,
                                                #   rht_num_candidates=NUM_CANDIDATES,
                                                  dbscan_eps=DBSCAN_EPS,
                                                  dbscan_min_samples=DBSCAN_MIN_SAMPLES,
                                                  save_org_events=UNFILTERED_FRAMES,
                                                  save_obstacles_img=SHOW_OBSTACLES,
                                                  estimate_depth=True,
                                                  draw_matches=True)

        input_data = np.frombuffer(msg.data, dtype=np.uint8)  # może będzie trochę szybsze
        self.__curr_frame_rgb = input_data.reshape([msg.height, msg.width, 3])
        self.__curr_frame = cv2.cvtColor(self.__curr_frame_rgb, cv2.COLOR_RGB2GRAY)

        self.__sim_time = (self.__ros2_node.get_clock().now().nanoseconds - self.__start_time) * REAL_TIME_FACTOR # nanoseconds

        if self.__gz_sim_time is not None:
            # print("gz ts:", DvsNode.gz_time_msg_to_float(self.__gz_sim_time))
            # print("ros ts:", self.__sim_time / 10 ** 9)
            events = self.__emulator.generate_events(self.__curr_frame, DvsNode.gz_time_msg_to_float(self.__gz_sim_time))
            # events = self.__emulator.generate_events(self.__curr_frame, self.__sim_time / 10 ** 9)  # sim_time to seconds
            # events: [[timestamp: float, x :int, y: int, polarity: int], [...], ...]
            if events is not None:
                if events.shape[0] < msg.height * msg.width / 3:  # Rejecting invalid frames
                    self.__events = events
                    self.__new_events = True
            
            # if events is not None:
            #     self.__detector.new_events(events)

    def events_cb(self):
        if self.__events is not None and self.__new_events:
            self.__new_events = False
            for event in self.__events:
                self.__detector.new_event(event)
            
    # def dvs_callback(self, msg: EventArray):
    #     ''' New packet of events '''
    #     events = []
    #     for event in msg.events:
    #         events.append(DvsEvent(event.x, event.y, event.ts.sec, event.ts.nanosec, event.polarity))

    #     event_array = DvsEventArray(msg.height, msg.width, events)
    #     self.__detector.new_events(event_array)

    def frame_timer_callback(self):
        ''' Show new frame '''
        if self.__detector is not None:
            filtered_frame = self.__detector.get_filtered_frame()
            
            # Displaying in separate with cv2
            # if self.__curr_frame is not None:
            #     cv2.imshow("camera", self.__curr_frame)
            #     cv2.waitKey(1)

            # if filtered_frame is not None:
            #     cv2.imshow("filtered_events", filtered_frame)
            #     cv2.waitKey(1)

            unfiltered_frame = self.__detector.get_unfiltered_frame()

            # if unfiltered_frame is not None:
            #     cv2.imshow("unfiltered_events", unfiltered_frame)
            #     cv2.waitKey(1)

            obstacles_img = self.__detector.get_obstacles_img()

            # if obstacles_img is not None:
            #     cv2.imshow("obstacles_img", obstacles_img)
            #     cv2.waitKey(1)

            if self.__curr_frame_rgb is not None and filtered_frame is not None and unfiltered_frame is not None and obstacles_img is not None:
                
                gap_width = 20

                height, _, _ = self.__curr_frame_rgb.shape
                white_gap = np.full((height, gap_width, 3), 255, dtype=np.uint8)

                row_with_gaps = cv2.hconcat([cv2.cvtColor(self.__curr_frame_rgb, cv2.COLOR_RGB2BGR), white_gap,
                                             cv2.cvtColor(unfiltered_frame, cv2.COLOR_GRAY2BGR), white_gap,
                                             cv2.cvtColor(filtered_frame, cv2.COLOR_GRAY2BGR), white_gap,
                                             obstacles_img])

                cv2.imshow("images", row_with_gaps)
                cv2.waitKey(1)

            


        # Publishing event frame
        # frame_msg = Image()
        # frame_msg.height = self.detector.frame_height
        # frame_msg.width = self.detector.frame_width
        # frame_msg.step = self.detector.frame_width * 1  # 1 is size of one pixel in bytes
        # frame_msg.data = filtered_frame.tobytes()
        # frame_msg.pixel_format_type = 1
        # self.__display_pub_.publish(frame_msg)
                 

def main(args=None):
    torch.set_grad_enabled(False)
    rclpy.init(args=args)

    node = DvsNode()
    try:
      while True:
        time.sleep(0.0001)
        pass
    except KeyboardInterrupt:
      pass
    # rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()
    print("\nDone")

if __name__=='__main__':
    main()