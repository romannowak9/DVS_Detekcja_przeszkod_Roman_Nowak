import numpy as np
import cv2
from typing import List, Tuple, Union, Optional
from collections import deque


# TODO: wrócić do wersji z timestmapami jako floatami, to jest wolne strasznie

# class TimeStamp():
#     def __init__(self, sec: Union[int, float], nsec: Optional[int] = None):
#         if nsec is not None:
#             self.sec = np.int32(sec)
#             self.nsec = np.uint32(nsec)
#         else:
#             self.sec, self.nsec = self.__float_to_sec_nsec(sec)

#     def __eq__(self, other):
#         return True if self.sec == other.sec and self.nsec == other.nsec else False

#     def __le__(self, other):
#         if self.sec < other.sec:
#             return True
#         elif self.sec > other.sec:
#             return False
#         else:
#             return True if self.nsec <= other.nsec else False

#     def __ge__(self, other):
#         if self.sec > other.sec:
#             return True
#         elif self.sec < other.sec:
#             return False
#         else:
#             return True if self.nsec >= other.nsec else False

#     def __lt__(self, other):
#         if self.sec < other.sec:
#             return True
#         elif self.sec > other.sec:
#             return False
#         else:
#             return True if self.nsec < other.nsec else False

#     def __gt__(self, other):
#         if self.sec > other.sec:
#             return True
#         elif self.sec < other.sec:
#             return False
#         else:
#             return True if self.nsec > other.nsec else False
        
#     def __sub__(self, other):
#         sec = self.sec - other.sec
#         nsec = np.int32(self.nsec) - np.int32(other.nsec)
#         if nsec < 0:
#             sec -= 1
#             nsec = 10 ** 9 + nsec

#         return TimeStamp(sec, nsec)

#     def __str__(self):
#         return "{}.{:09d}".format(self.sec, self.nsec)
        
#     def to_float(self) -> np.float64:
#         return self.sec + self.nsec / 10**9
    
#     def __float_to_sec_nsec(self, sec_f: float):
#         sec = int(sec_f)
#         nsec = (sec_f % 1) * 10 **9

#         return np.int32(sec), np.uint32(nsec)


def sec_to_nanosec(float_ts: float) -> np.int64:
    return np.int64(float_ts * 10 ** 9)

def nanosec_to_sec(nanosec) -> float:
    return nanosec / 10**9


class DvsEvent():
    def __init__(self, x: np.uint16, y: np.uint16, ts_sec: int, ts_nsec: int, polarity: bool):
        self.x = x
        self.y = y
        self.ts : np.int64 = ts_sec * 10 ** 9 + ts_nsec # nanosekundy, mogę użyć int64, bo wartość nanosekund mieści się w zakresie
        self.polarity = polarity


class DvsEventArray():
    def __init__(self, height: np.uint32, width: np.uint32, events: List[DvsEvent]):
        self.height = height
        self.width = width
        self.events = events


class DvsObstacleDetector():
    def __init__(self, frame_rate : int = 24, filter_t : float = 1 / 24, filter_k : int = 1, filter_size : int = 5):
        self.frame_height = 0
        self.frame_width = 0
        self.__events = deque()  # queue of events from last <timer_period> seconds

        # Filtering
        self.filter_t = sec_to_nanosec(filter_t)  # Flter’s correlation time threshold
        self.filter_k = filter_k  # Min number of neighbors
        self.filter_size = filter_size
        self.__SAE = None  # Surface of Active Events

        self.__frame_period = sec_to_nanosec(1 / frame_rate) # nanoseconds

        self.__filtered_events_frame = None

    def get_filtered_frame(self):
        return self.__filtered_events_frame

    def compute_output_frame(self):
        ''' Image representations of data. Must be called frame_rate time per second '''
        if self.frame_height and self.frame_width:
            ### EVENT FRAME AGREGATION ###
            # Event frame
            # dvs_frame = self.event_frame_agregation()

            # Exponentially decaying time surface
            self.__filtered_events_frame = self.__exponantial_decay_aggregation()  # wcześniej było: delta_t = DvsObstacleDetector.time_msg_to_float(msg.header.stamp) - DvsObstacleDetector.time_msg_to_float(self.prev_frame_ts

    def __exponantial_decay_aggregation(self) -> np.ndarray:
        ''' Type of event frame, wchich consider timestamp of event'''
        image_shape = (self.frame_height, self.frame_width)
        image = np.zeros(image_shape).astype(float) + 128.0

        if self.__events:
            events = list(self.__events)
            delta_t = self.__frame_period

            # I assume thet events are in ascending order by timestamps
            max = events[-1].ts
                    
            for event in events:
                image[event.y, event.x] = ((1 if event.polarity else -1) * 
                                        np.exp(- nanosec_to_sec(np.abs(max - event.ts) / delta_t)) + 1) * (255/2)
                #print(event[3] * np.exp(-np.abs(max - event[0]) / delta_t))

            # normalizedImg = np.zeros(image_shape)
            # normalizedImg = cv2.normalize(image,  normalizedImg, 0, 255, cv2.NORM_MINMAX)
            # normalizedImg = normalizedImg.astype(np.uint8)
            # print(f"max: {np.max(normalizedImg)}")
            # print(f"min: {np.min(normalizedImg)}")
            # print(f"1,1: {normalizedImg[1,1]}")

        # Displaying in separate with cv2
        # cv2.imshow("drone_dvs_exp_frame", normalizedImg)
        # cv2.waitKey(1)
        # cv2.imshow("drone_dvs_exp_frame_org", image.astype(np.uint8))
        # cv2.waitKey(1)

        return image.astype(np.uint8)

    def __event_frame_agregation(self) -> np.ndarray:
        ''' Standard event frame '''
        image_shape = (self.frame_height, self.frame_width)
        dvs_frame = (np.zeros(image_shape) + 125)

        if self.__events:
            events = list(self.__events)
            for event in events:
                if event.polarity:
                    dvs_frame[event.y, event.x] = 0
                else:
                    dvs_frame[event.y, event.x] = 255

        return dvs_frame.astype(np.uint8)

    def new_events(self, event_array: DvsEventArray):
        ''' Receiving new packet of events '''
        # print("new event packet")
        if event_array.events:
            self.frame_width = event_array.width
            self.frame_height = event_array.height

            events_to_remove = []

            for event in event_array.events:
                self.__update_SAE(event)
                # Filtering events
                if self.__is_BA(event):
                    events_to_remove.append(event)

            for event in events_to_remove:
                event_array.events.remove(event)

            if self.__events and event_array.events:
                # If need to be sure thet events are in order from the oldest the youngest
                # self.events.sort(key=lambda x: x.ts, reverse=False)

                # max_ts = max(event.ts for event in self.events)
                # I assume that Events in EventArray.events are in order from the oldest the youngest
                last_event_time = event_array.events[-1].ts
                oldest_event_time = self.__events[0].ts
                # Delete too old events
                while self.__events and (last_event_time - oldest_event_time) > self.__frame_period:
                    self.__events.popleft()

            self.__events.extend(event_array.events)
        
        # self.prev_frame_ts = msg.header.stamp
        
        # cv2.imshow("drone_dvs", np.array(list(frame_msg.data)).astype(np.uint8).reshape([msg.height, msg.width]))
        # cv2.waitKey(1)

    def __update_SAE(self, event: DvsEvent):
        ''' Updating Surface of Active Events (SAE) matrix '''
        # Exit if no events yet
        if not self.frame_height and not self.frame_width:
            return

        if self.__SAE is None:
            self.__SAE = np.zeros((self.frame_height, self.frame_width), dtype=np.int64)
        # W jednym artykule w tym miejscu odrzucają zbyt szybko występujące eventy o tej samej polaryzacji

        # I ignore polarity
        self.__SAE[event.y, event.x] = event.ts


    def __is_BA(self, event) -> bool:
        ''' 
            Filtering alghoritm - check if event is Background Activity or signal

            kNN - k nearest neighbors
        '''
        neighbors = -1  # to ignore considered event itself
        # Fixed window size
        # W jednym artykule używają to zmienego rozmiaru okna, obliczanego na podstawie optical flow
        offset = int(self.filter_size / 2.0)
        l_bound_y = event.y - offset
        u_bound_y = event.y + offset + 1
        l_bound_x = event.x - offset
        u_bound_x = event.x + offset + 1
        SAE_window = self.__SAE[l_bound_y if l_bound_y > 0 else 0 : u_bound_y if u_bound_y < self.frame_height else self.frame_height, 
                                l_bound_x if l_bound_x > 0 else 0 : u_bound_x if u_bound_x < self.frame_width else self.frame_width]
        
        event_ts = self.__SAE[event.y, event.x]
        for row in SAE_window:
            for neighbor_ts in row:
                if event_ts - neighbor_ts < self.filter_t:
                    neighbors += 1

        is_ba = False if neighbors >= self.filter_k else True
        return is_ba
