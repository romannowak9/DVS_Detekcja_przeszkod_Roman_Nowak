from v2ecore.emulator import EventEmulator
import cv2
import time
import torch


def main():
    # **IMPORTANT** make torch static, likely get faster emulation
    # might also cause memory issue
    torch.set_grad_enabled(False)

    emulator = EventEmulator(
            pos_thres=0.2,
            neg_thres=0.2,
            sigma_thres=0.03,
            cutoff_hz=200,
            leak_rate_hz=1,
            shot_noise_rate_hz=10,
            device="cuda")

    video_path = "ros2_px4_ws/src/drone_sim/tests/data/flower-garden.mp4"
    cap = cv2.VideoCapture(video_path)

    # num of frames
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("FPS: {}".format(fps))
    num_of_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print("Num of frames: {}".format(num_of_frames))

    duration = num_of_frames/fps
    delta_t = 1/fps
    current_time = 0.

    start_time = time.time()

    while True:
        # Read one frame
        ret, frame = cap.read()
        # Break the loop if no frame is captured
        if not ret:
            break
        
        # UWAGA! wartości timestamp nie mogą być zbyt duże!
        events = emulator.generate_events(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), time.time() - start_time)
        
        current_time += delta_t

        # print(time.time() - start_time)
        # print(current_time)

        
    # Release the video capture object
    cap.release()


if __name__ == '__main__':
    main()
