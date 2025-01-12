# python3 -m ros2_px4_ws.src.drone_sim.tests.detection_test

from ..modules.dvs_obstacles_detector import DvsObstacleDetector
import json
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

def sec_nsec_to_float(sec, nsec) -> float:
    return sec + nsec / 10**9


def test_detection(json_file_path, filter_t=1/200, filter_k=5, filter_size=5, frame_events_n=150):
    with open(json_file_path, 'r') as file:
        data = json.load(file)

    # if isinstance(data, list) and len(data) > 0:
    #     print(len(data))
    #     print(data[0].keys())
    #     print(len(data[0]['Events']))
    #     print(data[0]['Events'][0].keys())
    #     print(type(data[0]['Events'][0]['Polarity']))

    # prev_ts = None
    start_sec = data[0]['Events'][0]['Ts']['Sec']
    height = data[0]['Height']
    width = data[0]['Width']

    event_arrays = np.empty(len(data), dtype=np.ndarray)
    for i, event_array_dict in enumerate(data):
        # ts = (event_array_dict['Events'][0]['Ts']['Sec'], event_array_dict['Events'][0]['Ts']['Nsec'])
        # if prev_ts is not None:
        #     # print(ts - prev_ts)
        #     print(ts)
        # prev_ts = ts

        events = np.empty(len(event_array_dict['Events']), dtype=tuple)
        for j, event_dict in enumerate(event_array_dict['Events']):
            event = (sec_nsec_to_float(event_dict['Ts']['Sec'] - start_sec, event_dict['Ts']['Nsec']), event_dict['X'], event_dict['Y'], 1 if event_dict['Polarity'] else -1)
            events[j] = event
        
        event_arrays[i] = events

    detector = DvsObstacleDetector(frame_shape=(height, width),
                                   n_frame_events=frame_events_n,
                                   filter_t=filter_t,
                                   filter_k=filter_k,
                                   filter_size=filter_size,
                                #    rht_samples=2000,
                                #    rht_bins=0.5,
                                #    rht_min_votes=3,
                                #    rht_min_plane_points=50,  
                                #    rht_num_candidates=8,
                                #    rht_min_point_dist=0.5,
                                   dbscan_eps=20,
                                   dbscan_min_samples=25,
                                   save_org_events=True,
                                   save_obstacles_img=True)

    for event_array in event_arrays:
        for event in event_array:
            #print(event)
            detector.new_event(event)

        filtered_frame = detector.get_filtered_frame()

        if filtered_frame is not None:
            cv2.imshow("filtered_events", filtered_frame)
            cv2.waitKey(1)

        unfiltered_frame = detector.get_unfiltered_frame()

        if unfiltered_frame is not None:
            cv2.imshow("unfiltered_events", unfiltered_frame)
            cv2.waitKey(1)

        obstacles_img = detector.get_obstacles_img()

        if obstacles_img is not None:
            cv2.imshow("obstacles_img", obstacles_img)
            cv2.waitKey(1)

        cv2.waitKey(0)

        # events = detector.get_filtered_events()
        # events = np.array(events)[:, :3]

        # fig = plt.figure()
        # ax = fig.add_subplot(projection='3d')
        # ax.scatter(events[:,1], events[:,0], events[:,2], marker='*')
        # ax.set_xlabel('x')
        # ax.set_ylabel('ts')
        # ax.set_zlabel('y')

        # # Render the plot to an image
        # canvas = FigureCanvas(fig)
        # canvas.draw()
        # plot_image = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        # plot_image = plot_image.reshape(canvas.get_width_height()[::-1] + (3,))

        # # Convert RGB (Matplotlib) to BGR (OpenCV)
        # plot_image = cv2.cvtColor(plot_image, cv2.COLOR_RGB2BGR)

        # # Display the plot in an OpenCV window
        # cv2.imshow("Events", plot_image)
        # cv2.waitKey(0)

def main():
    json_file_path = 'ros2_px4_ws/src/drone_sim/tests/data/shapes_rotation.json'
    test_detection(json_file_path, filter_t=1/24, filter_k=3, filter_size=3, frame_events_n=500)

    # json_file_path = 'ros2_px4_ws/src/drone_sim/tests/data/urban_events.json'
    # test_detection(json_file_path, filter_t=1/24, filter_k=3, filter_size=3, frame_events_n=300)

    # json_file_path = 'ros2_px4_ws/src/drone_sim/tests/data/outdoors_walking_events.json'
    # test_detection(json_file_path, filter_t=1/24, filter_k=2, filter_size=3, frame_events_n=300)

    json_file_path = 'ros2_px4_ws/src/drone_sim/tests/data/night_run_events.json'
    test_detection(json_file_path, filter_t=1/400, filter_k=6, filter_size=3, frame_events_n=500)

    # json_file_path = 'ros2_px4_ws/src/drone_sim/tests/data/night_drive_events.json'
    # test_detection(json_file_path, filter_t=1/24, filter_k=6, filter_size=3, frame_events_n=3000)



if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
