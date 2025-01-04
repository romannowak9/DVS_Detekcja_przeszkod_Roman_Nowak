import numpy as np
import cv2
from typing import List, Tuple, Union, Optional, Dict
from collections import deque
from copy import deepcopy, copy
import random
from sklearn.cluster import DBSCAN
from sort_tracker import Sort

# from modules.rht import rht_planes, rht_planes2


# TODO: Spróbować z innymi topicami na R i t, sprawdzić częstotliwości topiców, własna funkcja do estymacji na podst. https://www.youtube.com/watch?v=Qm7vunJAtKY (Warto spróbować - inne podejście)

event_field = {'ts': 0, 'x': 1, 'y': 2, 'polarity': 3}


class Obstacle():
    def __init__(self, events_locs: np.ndarray, ts: float, camera_pos, camera_rot):
        self.locs = events_locs
        self.contour = cv2.convexHull(events_locs.astype(np.int32).reshape(-1, 1, 2))

        hsv_color = [random.randint(0, 179),  # H
                     255,  # S
                     255]  # V
        
        hsv_color = np.uint8([[hsv_color]])
        self.color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0][0]

        self.__compute_area_and_centroid()

        self.v = None
        self.distance = None

        self.confidence_score = 1
        self.bounds = np.array([np.min(events_locs[:,0]), np.min(events_locs[:,1]), np.max(events_locs[:,0]), np.max(events_locs[:,1])])  # [x_min, y_min, x_max, y_max]

        self.ts = ts
        # self.updated = True

        self.obstacle_idx = None

        self.camera_pos = camera_pos
        self.camera_rot = camera_rot

    def get_frame(self, h: int, w: int):

        frame = np.zeros((h,w), dtype=np.uint8)

        for (x, y) in self.locs:
            frame[y, x] = np.uint8(255)

        return frame

    def __compute_area_and_centroid(self):
        moments = cv2.moments(self.contour)

        area = moments['m00']

        if area != 0:
            centroid_x = int(moments['m10'] / area)
            centroid_y = int(moments['m01'] / area)
        else:
            centroid_x, centroid_y = 0, 0

        # print(f"Area of the filled contour: {area}")
        # print(f"Centroid of the filled contour: ({centroid_x}, {centroid_y})")

        self.area = area
        self.centroid = np.array((centroid_x, centroid_y))

    def draw(self, image: np.ndarray) -> np.ndarray:
        '''
        Draws obstacle on given bgr image
        '''
        cv2.fillPoly(image, [self.contour], color=(int(self.color[0]), int(self.color[1]), int(self.color[2])))
        # for x, y in self.locs:
        #     y = int(y)
        #     x = int(x)
        #     image[y,x,0] = self.color[0]
        #     image[y,x,1] = self.color[1]
        #     image[y,x,2] = self.color[2]

        cross_color = (255, 255, 255)
        cv2.line(image, (self.centroid[0] - 2, self.centroid[1] - 2), (self.centroid[0] + 2, self.centroid[1] + 2), cross_color, 1)
        cv2.line(image, (self.centroid[0] + 2, self.centroid[1] - 2), (self.centroid[0] - 2, self.centroid[1] + 2), cross_color, 1)

        # image[self.centroid[1], self.centroid[0]] = [255, 255, 255]
        # image[self.centroid[1] + 1, self.centroid[0]] = [255, 255, 255]
        # image[self.centroid[1], self.centroid[0] + 1] = [255, 255, 255]
        # image[self.centroid[1] - 1, self.centroid[0]] = [255, 255, 255]
        # image[self.centroid[1], self.centroid[0] - 1] = [255, 255, 255]

    def __eq__(self, other):
        return True if self.obstacle_idx == other.obstacle_idx else False
        
    # def update(self, events_locs):
    #     self.locs = events_locs
    #     self.contour = cv2.convexHull(events_locs.astype(np.int32).reshape(-1, 1, 2))
    #     self.__compute_area_and_centroid()

    #     self.updated = True



class DvsObstacleDetector():
    def __init__(self, frame_shape : Tuple, n_frame_events : int = 500,  # Events params
                 filter_t : float = 1 / 24, filter_k : int = 1, filter_size : int = 5,  # Filter params
                #  rht_samples : Optional[int] = None, rht_bins : int = 20, rht_min_votes : int = 3,  # RHT params
                #  rht_min_plane_points : Optional[int] = None, rht_num_candidates : int = 5, rht_min_point_dist: float = 0.5,  # RHT params
                 dbscan_eps : int = 3, dbscan_min_samples : Optional[int] = None,  # DBSCAN
                 estimate_depth = False,
                 save_org_events : bool = False, save_obstacles_img=False, draw_matches=False):  # Visualization params
        
        self.frame_height = frame_shape[0]
        self.frame_width = frame_shape[1]

        self.__filtered_events = []
        self.__original_events = []

        self.__original_events_buff = []
        self.__original_events_buff = []

        # Filtering
        self.filter_t = filter_t  # Flter’s correlation time threshold
        self.filter_k = filter_k  # Min number of neighbors
        self.filter_size = filter_size  # Filter window size
        self.__SAE = None  # Surface of Active Events

        self.__frame_period = None
        self.n = n_frame_events

        self.__filtered_events_frame = None
        self.__original_events_frame = None

        # RHT
        # self.rht_samples = (4 * n_frame_events) if rht_samples is None else rht_samples
        # self.rht_bins = rht_bins
        # self.rht_min_votes = rht_min_votes
        # self.rht_min_plane_points = int(n_frame_events / 10) if rht_min_plane_points is None else rht_min_plane_points
        # self.rht_num_candidates = rht_num_candidates
        # self.rht_min_point_dist = rht_min_point_dist

        # DBSCAN
        self.db_eps = dbscan_eps
        self.db_min_samples = int(n_frame_events / 10) if dbscan_min_samples is None else dbscan_min_samples

        self.__dbscan = DBSCAN(eps=self.db_eps, min_samples=self.db_min_samples)

        # SORT tracker
        self.__tracker = Sort()

        # Depth estimation
        self.estimate_depth = estimate_depth
        self.__curr_cam_pos : np.ndarray = None
        self.__curr_cam_rot : np.ndarray = None

        self.__orb = cv2.ORB_create()
        self.__bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Visualization
        self.save_org_events = save_org_events
        self.save_obstacles_img = save_obstacles_img
        self.obstacles_img = None
        self.draw_matches = draw_matches

        self.__obstacles_by_idxs: Dict[int, Obstacle] = {}
        self.__curr_obstacles: List[Obstacle] = []
        self.__prev_obstacles_by_idxs: Dict[int, Obstacle] = {}

    def update_camera_pos(self, position, rotation):
        self.__curr_cam_pos = np.array(position)
        self.__curr_cam_rot = np.array(rotation)

    def get_filtered_events(self):
        return self.__filtered_events

    def get_filtered_frame(self):
        return self.__filtered_events_frame
    
    def get_unfiltered_frame(self):
        return self.__original_events_frame
    
    def get_obstacles_img(self):
        return self.obstacles_img

    def compute_output_frame(self, events, delta_t: float):
        ''' Image representations of data. Must be called frame_rate time per second '''
        ### EVENT FRAME AGREGATION ###
        # Event frame
        # dvs_frame = self.event_frame_agregation()

        # Exponentially decaying time surface
        dvs_frame = self.__exponantial_decay_aggregation(events, delta_t=delta_t)  # wcześniej było: delta_t = DvsObstacleDetector.time_msg_to_float(msg.header.stamp) - DvsObstacleDetector.time_msg_to_float(self.prev_frame_ts

        return dvs_frame

    def new_event_frame(self):
        ''' Processing of filtered and sliced events from self.__to_frame_filtered_events '''
        
        # RHT

        # events = np.array(self.__filtered_events)[:, :3]
        # obstacles = rht_planes(events,
        #                     num_samples=self.rht_samples,
        #                     bins=self.rht_bins,
        #                     min_votes=self.rht_min_votes,
        #                     min_plane_points=self.rht_min_plane_points,
        #                     num_candidates=self.rht_num_candidates,
        #                     min_distance=self.rht_min_point_dist)
        
        # obstacles = rht_planes2(events,
        #                      num_samples=self.rht_samples,
        #                      bins=self.rht_bins,
        #                      min_plane_points=self.rht_min_plane_points,
        #                      min_distance=self.rht_min_point_dist)

        # print(list(obstacles.keys()))
        # print([el.shape[0] for el in list(obstacles.values())])

        # \RHT

        # DBSCAN
        events = np.array(self.__filtered_events)[:, :3]  # Only x and y - ignoring polarity
        labels = self.__dbscan.fit_predict(events)
        obstacles_events = list()

        for label in set(labels):
            if label == -1:  # Noise
                # print("DBSCAN noise:", len(events[labels==label]))
                pass
            else:
                obstacles_events.append(events[labels == label])

        # print("Number of clusters:", len(obstacles_events))
        # print([o_events.shape[0] for o_events in obstacles_events])

        # Displaying obstacles
        if self.save_obstacles_img:
            self.obstacles_img = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)

        # There is obstacle/-s
        # for plane, o_events in obstacles.items():  # RHT
        new_obstacles: List[Obstacle] = []
        detections = np.empty((len(obstacles_events), 5))
        for i, o_events in enumerate(obstacles_events):
            o_events_locs = o_events[:, 1:].astype(int)
            new_obstacle = Obstacle(o_events_locs, ts=np.max(o_events[:,0]), camera_pos=self.__curr_cam_pos, camera_rot=self.__curr_cam_rot)
            detections[i] = np.append(new_obstacle.bounds, new_obstacle.confidence_score)

            new_obstacles.append(new_obstacle)

        detections = np.array(detections)
        # print("detections:", detections)
        tracked_obstacles = self.__tracker.update(dets=detections)
        # print("tracked:", tracked_obstacles)

        self.__prev_obstacles_by_idxs = deepcopy(self.__obstacles_by_idxs)

        # TODO: może to by trzeba przerobić na odwrotną pętlę - niech każda śledzona dostaje obiekt z updated_obstacles
        # new_bounds = [obstacle.bounds for obstacle in updated_obstacles]

        # for tracked_obstacle in tracked_obstacles:
        #     tracked_bounds = tracked_obstacle[:4]
        #     tracked_idx = tracked_obstacle[5]

        #     closest_obj_idx = DvsObstacleDetector.__get_closest_array_idx(tracked_bounds, new_bounds)
        #     closest_new_obstacle = updated_obstacles[closest_obj_idx]

        #     closest_new_obstacle.obstacle_idx = tracked_idx

        #     if tracked_idx in self.__obstacles_by_idxs:
        #         # Old color but new obstacle
        #         closest_new_obstacle.color = self.__obstacles_by_idxs[tracked_idx].color

        #     self.__obstacles_by_idxs[tracked_idx] = closest_new_obstacle

        updated_obstacles : List[Obstacle] = list()
        for new_obstacle in new_obstacles:
            if tracked_obstacles.shape[0] > 0:
                # Getting idx
                closest_idx = DvsObstacleDetector.__get_closest_array_idx(new_obstacle.bounds, tracked_obstacles[:,:4])
                obstacle_idx = tracked_obstacles[closest_idx, 5]
                tracked_obstacles = np.delete(tracked_obstacles, closest_idx, axis=0)
                new_obstacle.obstacle_idx = obstacle_idx

                if obstacle_idx in self.__obstacles_by_idxs:
                    # Old color but new obstacle
                    new_obstacle.color = self.__obstacles_by_idxs[obstacle_idx].color

                # Update obstacle in dict
                self.__obstacles_by_idxs[obstacle_idx] = new_obstacle

                updated_obstacles.append(new_obstacle)

        # Update current obstacles
        self.__curr_obstacles = updated_obstacles

        # Visualize
        if self.save_obstacles_img:
            for obstacle in self.__curr_obstacles:
                obstacle.draw(self.obstacles_img)

        # Depth estimation
        if self.estimate_depth:
            # print("cam_pos:", self.__curr_cam_pos)
            # print("cam_rot:", self.__curr_cam_rot)
            # Intrinsic matrixs
            K = np.array([
                [207.89351463317871, 0, 120],
                [0, 207.89351463317871, 90],
                [0, 0, 1]
            ])
            dist = np.array([0,0,0,0,0])

            for obstacle in self.__curr_obstacles:
                curr_pos = obstacle.camera_pos
                curr_rot = obstacle.camera_rot
                curr_ts = obstacle.ts
                curr_obst_frame = obstacle.get_frame(self.frame_height, self. frame_width)

                obst_idx = obstacle.obstacle_idx
                if obst_idx not in self.__prev_obstacles_by_idxs:
                    continue

                prev_pos = self.__prev_obstacles_by_idxs[obst_idx].camera_pos
                prev_rot = self.__prev_obstacles_by_idxs[obst_idx].camera_rot
                prev_ts = self.__prev_obstacles_by_idxs[obst_idx].ts
                prev_obst_frame = self.__prev_obstacles_by_idxs[obst_idx].get_frame(self.frame_height, self. frame_width)

                # print(f"prev_ts: {prev_ts}; curr_ts: {curr_ts}")

                # Computing distances - triangulation

                curr_rot = DvsObstacleDetector.normalize_quaternion(curr_rot)
                prev_rot = DvsObstacleDetector.normalize_quaternion(prev_rot)

                curr_R = DvsObstacleDetector.quaternion_to_rotation_matrix(curr_rot)
                prev_R = DvsObstacleDetector.quaternion_to_rotation_matrix(prev_rot)

                # Projection matrices
                curr_P = K @ np.hstack((curr_R, curr_pos.reshape((3,1))))
                prev_P = K @ np.hstack((prev_R, prev_pos.reshape((3,1))))

                # Matching points - ORB
                # Detect keypoints and compute descriptors
                curr_keypoints, curr_descriptors = self.__orb.detectAndCompute(curr_obst_frame, None)
                prev_keypoints, prev_descriptors = self.__orb.detectAndCompute(prev_obst_frame, None)

                if len(curr_keypoints) < 4 or len(prev_keypoints) < 4:
                    continue

                matches = self.__bf.match(curr_descriptors, prev_descriptors)
                matches = sorted(matches, key=lambda x: x.distance)

                curr_matched_points = np.array([curr_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                prev_matched_points = np.array([prev_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
                
                if curr_matched_points.shape[0] < 4:
                    continue

                _, mask = cv2.findHomography(curr_matched_points, prev_matched_points, cv2.RANSAC, 0.1)
                inliers_matches = [matches[i] for i in range(len(matches)) if mask[i]]

                if not mask.any():
                    continue

                if self.draw_matches:
                    # Draw inlier matches
                    img_inliers = cv2.drawMatches(
                        curr_obst_frame, curr_keypoints, prev_obst_frame, prev_keypoints, inliers_matches, None, matchColor=(0, 255, 0), flags=2
                    )
                    cv2.imshow("inliers_matches", img_inliers)
                    cv2.waitKey(1)

                # Filter only inliers using the mask
                curr_matched_points = curr_matched_points[mask.ravel() == 1]
                prev_matched_points = prev_matched_points[mask.ravel() == 1]

                p3ds = []
                for uv1, uv2 in zip(curr_matched_points.reshape(-1,2), prev_matched_points.reshape(-1,2)):
                    _p3d = DvsObstacleDetector.DLT(curr_P, prev_P, uv1, uv2)
                    p3ds.append(_p3d)
                p3ds = np.array(p3ds)

                print("DLT")
                print(f"[{obstacle.obstacle_idx}] : dist = {np.min(p3ds)}")
                print("distances:", p3ds)

                # Undistort to narmalize and prepare for triangulation
                curr_matched_points = cv2.undistortPoints(curr_matched_points, K, dist)
                prev_matched_points = cv2.undistortPoints(prev_matched_points, K, dist)

                # Triangulation
                points = cv2.triangulatePoints(curr_P, prev_P, curr_matched_points, prev_matched_points)
                # Convert homogeneous coordinates to 3D
                points = points[:3, :] / points[3, :]
                # Extract Z-coordinate (depth)
                distances = points[2, :]  # Z-coordinates are the distances

                obstacle.distance = np.min(distances)  # Distance to the closest point of the obstacle

                print("OpenCV")
                print(f"[{obstacle.obstacle_idx}] : dist = {obstacle.distance}")
                print("distances:", distances)
                
                # Computing velocities towards vehicle
                curr_distance = obstacle.distance
                prev_distance = self.__prev_obstacles_by_idxs[obst_idx].distance
                if prev_distance is None:
                    continue

                curr_v = (prev_distance - curr_distance) / (curr_ts - prev_ts)  # m(?)/s

                obstacle.v = curr_v

                # print(f"[{obstacle.obstacle_idx}] : vel = {obstacle.v}")

                p3ds = []
                for uv1, uv2 in zip(curr_matched_points.reshape(-1,2), prev_matched_points.reshape(-1,2)):
                    _p3d = DvsObstacleDetector.DLT(curr_P, prev_P, uv1, uv2)
                    p3ds.append(_p3d)
                p3ds = np.array(p3ds)

                print("DLT Undistorted")
                print(f"[{obstacle.obstacle_idx}] : dist = {np.min(p3ds)}")
                print("distances:", p3ds)


    @staticmethod
    def DLT(P1, P2, point1, point2):

        A = [point1[1]*P1[2,:] - P1[1,:],
            P1[0,:] - point1[0]*P1[2,:],
            point2[1]*P2[2,:] - P2[1,:],
            P2[0,:] - point2[0]*P2[2,:]
            ]
        A = np.array(A).reshape((4,4))
        #print('A: ')
        #print(A)
    
        B = A.transpose() @ A
        from scipy import linalg
        U, s, Vh = linalg.svd(B, full_matrices = False)
    
        # print('Triangulated point: ')
        # print(Vh[3,0:3]/Vh[3,3])
        return Vh[3,0:3]/Vh[3,3]


    @staticmethod
    def normalize_quaternion(q: np.ndarray):
        return q / np.linalg.norm(q)

    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """
        Convert a quaternion [x, y, z, w] to a 3x3 rotation matrix.
        """
        # q_x, q_y, q_z, q_w = q
        # R = np.array([
        #     [1 - 2 * (q_y**2 + q_z**2), 2 * (q_x * q_y - q_w * q_z), 2 * (q_x * q_z + q_w * q_y)],
        #     [2 * (q_x * q_y + q_w * q_z), 1 - 2 * (q_x**2 + q_z**2), 2 * (q_y * q_z - q_w * q_x)],
        #     [2 * (q_x * q_z - q_w * q_y), 2 * (q_y * q_z + q_w * q_x), 1 - 2 * (q_x**2 + q_y**2)]
        # ])

        q1, q2, q3, q0 = q
        # https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
        R = np.array([
            [2 * (q0**2 + q1**2) - 1, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 2 * (q0**2 + q2**2) - 1, 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 2 * (q0**2 + q3**2) - 1]
        ])
        return R

    @staticmethod
    def __get_closest_array_idx(array, arrays_to_choose):
        distances = np.linalg.norm(arrays_to_choose - array, axis=1)
        closest_idx = np.argmin(distances)

        return closest_idx

    def __exponantial_decay_aggregation(self, events, delta_t: float) -> np.ndarray:
        ''' Type of event frame, wchich consider timestamp of event'''
        image_shape = (self.frame_height, self.frame_width)
        image = np.zeros(image_shape).astype(float) + 128.0

        if events and delta_t > 0:
            #events = list(events)

            # I assume thet events are in ascending order by timestamps
            max = events[-1][event_field['ts']]  # ts
                    
            for event in event:
                image[int(event[event_field['y']]), int(event[event_field['x']])] = (event[event_field['polarity']] * 
                                        np.exp(- np.abs(max - event[event_field['ts']]) / delta_t) + 1) * (255/2)

        elif events and delta_t == 0:
            for event in events:
                image[int(event[event_field['y']]), int(event[event_field['x']])] = 255

        # Displaying in separate with cv2
        # cv2.imshow("drone_dvs_exp_frame", normalizedImg)
        # cv2.waitKey(1)
        # cv2.imshow("drone_dvs_exp_frame_org", image.astype(np.uint8))
        # cv2.waitKey(1)

        return image.astype(np.uint8)

    def __event_frame_agregation(self, events) -> np.ndarray:
        ''' Standard event frame '''
        image_shape = (self.frame_height, self.frame_width)
        dvs_frame = (np.zeros(image_shape) + 125)

        if events:
            # events = list(events)
            for event in events:
                if event[event_field['polarity']] > 0:
                    dvs_frame[int(event[event_field['y']]), int(event[event_field['x']])] = 0
                else:
                    dvs_frame[int(event[event_field['y']]), int(event[event_field['x']])] = 255

        return dvs_frame.astype(np.uint8)

    def new_event(self, event):
        '''
        Receiving a new sigle event in format [timestamp: float, x :int, y: int, polarity: int]
        '''

        event[1] = int(event[1])
        event[2] = int(event[2])
        event[3] = int(event[3])

        if self.save_org_events:
            self.__original_events.append(event)
            if len(self.__original_events) >= self.n:
                delta_t = self.__original_events[-1][event_field['ts']] - self.__original_events[0][event_field['ts']]
                self.__original_events_frame = self.compute_output_frame(self.__original_events, delta_t)
                self.__original_events = []
                # I dont precess this one - it is only for visualization

        self.__update_SAE(event)

        if self.__is_BA(event):
            return
        
        self.__filtered_events.append(event)

        if len(self.__filtered_events) >= self.n:
            self.__frame_period = self.__filtered_events[-1][event_field['ts']] - self.__filtered_events[0][event_field['ts']]
            
            self.__filtered_events_frame = self.compute_output_frame(self.__filtered_events, delta_t=self.__frame_period)
            # Processing events
            self.new_event_frame()
            # Jakbym chciał to jakoś zrównoleglić, to trzeba będzie wrócić do wersji z buforami
            self.__filtered_events = []


    # def new_events(self, event_array):
    #     ''' 
    #     Receiving new packet of events
    #     events: [[timestamp: float, x :int, y: int, polarity: int], [...], ...]
    #     '''
    #     # TODO: A może lepiej będzie brać pojedyńcze eventy? Żeby zbieranie n eventów miało sens - trzeba będzie sprawdzić

    #     if not isinstance(event_array, List):
    #         event_array = list(event_array)

    #     # print("new event packet")
    #     if event_array:
    #         if self.save_org_events:
    #             org_events = deepcopy(event_array)

    #         idxs_to_remove = []
    #         for event in event_array:
    #             self.__update_SAE(event)
            
    #         for i, event in enumerate(event_array):
    #             # Filtering events
    #             if self.__is_BA(event):
    #                 idxs_to_remove.append(i)

    #         for idx in idxs_to_remove[::-1]:
    #             del event_array[idx]

    #         signal_events_n = len(event_array)
    #         ba_events_n = len(idxs_to_remove)
    #         all_events_n = signal_events_n + ba_events_n
    #         print(f"signal: {signal_events_n}, ba: {ba_events_n}")
            
    #         self.__filtered_events_buff.extend(event_array)
    #         if signal_events_n > self.n:
    #             print("Too many signal events in one packet!")
    #             self.__filtered_events = self.__filtered_events_buff  # event_array[-self.n:]
    #             self.__filtered_events_buff = []
    #             self.__frame_period = event_array[-1][event_field['ts']] - event_array[0][event_field['ts']]
    #             self.__filtered_events_frame = self.compute_output_frame(self.__filtered_events)
    #             # Processing events
    #             self.new_event_frame()
    #         else:
    #             if len(self.__filtered_events_buff) >= self.n:
    #                 self.__filtered_events = self.__filtered_events_buff[:self.n]
    #                 self.__filtered_events_buff = self.__filtered_events_buff[self.n:]
    #                 self.__frame_period = self.__filtered_events[-1][event_field['ts']] - self.__filtered_events[0][event_field['ts']]
    #                 self.__filtered_events_frame = self.compute_output_frame(self.__filtered_events)
    #                 # Processing events
    #                 self.new_event_frame()

    #         # Same for unfiltered if desired
    #         if self.save_org_events:
    #             self.__original_events_buff.extend(org_events)
    #             if all_events_n > self.n:
    #                 self.__original_events = self.__original_events_buff
    #                 self.__original_events_buff = []
    #                 # self.__frame_period = org_events[-1][event_field['ts']] - org_events[0][event_field['ts']]
    #                 self.__original_events_frame = self.compute_output_frame(self.__original_events)
    #                 # I dont process this one - it is only for visualization
    #             else:
    #                 if len(self.__original_events_buff) >= self.n:
    #                     self.__original_events = self.__original_events_buff[:self.n]
    #                     self.__original_events_buff = self.__original_events_buff[self.n:]
    #                     # self.__frame_period = self.__to_frame_original_events[-1][event_field['ts']] - self.__to_frame_original_events[0][event_field['ts']]
    #                     self.__original_events_frame = self.compute_output_frame(self.__original_events)
    #                     # I dont process this one - it is only for visualization

    def __update_SAE(self, event):
        ''' Updating Surface of Active Events (SAE) matrix '''
        # Exit if no events yet
        if not self.frame_height or not self.frame_width:
            return

        if self.__SAE is None:
            self.__SAE = np.zeros((self.frame_height, self.frame_width), dtype=float)
        # W jednym artykule w tym miejscu odrzucają zbyt szybko występujące eventy o tej samej polaryzacji

        # I ignore polarity
        self.__SAE[int(event[event_field['y']]), int(event[event_field['x']])] = event[event_field['ts']]

    def __is_BA(self, event) -> bool:
        ''' 
            Filtering alghoritm - check if event is Background Activity or signal

            kNN - k nearest neighbors
        '''
        neighbors = -1  # to ignore considered event itself
        # Fixed window size
        # W jednym artykule używają to zmienego rozmiaru okna, obliczanego na podstawie optical flow
        offset = int(self.filter_size / 2.0)
        l_bound_y = int(event[event_field['y']] - offset)
        u_bound_y = int(event[event_field['y']] + offset + 1)
        l_bound_x = int(event[event_field['x']] - offset)
        u_bound_x = int(event[event_field['x']] + offset + 1)
        SAE_window = self.__SAE[l_bound_y if l_bound_y > 0 else 0 : u_bound_y if u_bound_y < self.frame_height else self.frame_height, 
                                l_bound_x if l_bound_x > 0 else 0 : u_bound_x if u_bound_x < self.frame_width else self.frame_width]
        
        event_ts = self.__SAE[int(event[event_field['y']]), int(event[event_field['x']])]
        for row in SAE_window:
            for neighbor_ts in row:
                if event_ts - neighbor_ts < self.filter_t:
                    neighbors += 1

        is_ba = False if neighbors >= self.filter_k else True
        return is_ba
