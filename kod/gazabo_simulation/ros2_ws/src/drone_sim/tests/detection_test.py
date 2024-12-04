# python3 -m ros2_px4_ws.src.drone_sim.tests.detection_test

from ..modules.dvs_obstacles_detector import DvsObstacleDetector, DvsEventArray, DvsEvent
import json
import cv2


def time_dict_to_float(time_msg: dict) -> float:
    return time_msg['Sec'] + time_msg['Nsec'] / 10**9


def test_detection(json_file_path, filter_t=1/200, filter_k=5, filter_size=5, frame_rate=33):
    with open(json_file_path, 'r') as file:
        data = json.load(file)

    # if isinstance(data, list) and len(data) > 0:
    #     print(len(data))
    #     print(data[0].keys())
    #     print(len(data[0]['Events']))
    #     print(data[0]['Events'][0].keys())
    #     print(type(data[0]['Events'][0]['Polarity']))

    event_arrays = []
    # prev_ts = None
    for event_array_dict in data:

        # ts = (event_array_dict['Events'][0]['Ts']['Sec'], event_array_dict['Events'][0]['Ts']['Nsec'])
        # if prev_ts is not None:
        #     # print(ts - prev_ts)
        #     print(ts)
        # prev_ts = ts

        events = []
        for event_dict in event_array_dict['Events']:
            event = DvsEvent(event_dict['X'], event_dict['Y'], event_dict['Ts']['Sec'], event_dict['Ts']['Nsec'], event_dict['Polarity'])
            events.append(event)
        
        event_array = DvsEventArray(event_array_dict['Height'], event_array_dict['Width'], events)
        event_arrays.append(event_array)

    detector = DvsObstacleDetector(frame_rate=frame_rate, filter_t=filter_t, filter_k=filter_k, filter_size=filter_size)

    for event_array in event_arrays:
        detector.new_events(event_array)
        detector.compute_output_frame()

        filtered_frame = detector.get_filtered_frame()

        if filtered_frame is not None:
            cv2.imshow("filtered_events", filtered_frame)
            cv2.waitKey(0)


def main():
    json_file_path = 'ros2_px4_ws/src/drone_sim/tests/data/shapes_rotation.json'
    test_detection(json_file_path, filter_t=1/200, filter_k=5, filter_size=5, frame_rate=33)

    # json_file_path = 'ros2_px4_ws/src/drone_sim/tests/data/urban_events.json'
    # test_detection(json_file_path, filter_t=1/200, filter_k=5, filter_size=5, frame_rate=33)

    # json_file_path = 'ros2_px4_ws/src/drone_sim/tests/data/outdoors_walking_events.json'
    # test_detection(json_file_path, filter_t=1/200, filter_k=5, filter_size=5, frame_rate=33)


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
