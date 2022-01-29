import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import logging
from picarx_improved import Picarx
from camera import Camera
import cv2
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.INFO)

if __name__ == "__main__":
    px = Picarx()
    camera = Camera()
    
    # Tilting the camera down
    px.set_camera_servo2_angle(-10)

    logging.info("Start lane following.\n")
    for frame in camera.camera.capture_continuous(camera.raw_cap, format="bgr", use_video_port=True):

        frame_array = frame.array

        lane_lines = camera.detect_lane(frame_array)
        lines_img = camera.display_lane_lines(frame_array, lane_lines)

        steering_angle = camera.get_steering_angle(frame_array, lane_lines)
        heading_img = camera.display_heading_line(lines_img, steering_angle)
        cv2.imshow('heading', heading_img)

        camera.raw_cap.truncate(0)

        px.set_dir_servo_angle(steering_angle - 90)
        px.forward(30)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            cv2.destroyAllWindows()
            camera.camera.close()
            px.stop()
            break
