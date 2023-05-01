"""Main script to run the object detection routine."""
import argparse
import sys
import time
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils
import serial
import struct
import math

from distance_sampler import DistanceSampler2

ser = serial.Serial('/dev/ttyAMA0',9600,timeout=None)

def thread_test():
    while(1):
        #ser.write(b"Hello\n");
        test_str = "Hello\n"
        ser.write(test_str.encode('utf-8'))
        time.sleep(1)

def run(
    model: str,
    camera_id: int,
    width: int,
    height: int,
    num_threads: int,
    enable_edgetpu: bool,
    visualize_image: bool
) -> None:


  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  # Initialize the object detection model
  base_options = core.BaseOptions(file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
  detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
  options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)

  # pixel_size = 398.0      #3.98 um
  pixel_size = 304.0

  focalLength = 3.67     # in mm
  # knownDistance = 50      # in mm
  knownDistance = 100.0
  absoluteWidth = 49.784  # 1.96 in
  absoluteHeight = 49.784 # 1.96 in
  distanceSampler = DistanceSampler2(
        float(pixel_size),
        float(knownDistance),
        float(absoluteWidth),
        float(absoluteHeight)
    )
  
  # Continuously capture images from the camera and run inference
  while cap.isOpened():

    success, image = cap.read()
    if not success:
        raise Exception("Can't read from Webcam")

    counter += 1

    # Preprocess image for EfficientNet and create tensor image
    image = cv2.flip(image, 1)
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection estimation using the model.
    detection_result = detector.detect(input_tensor)
    
    # Check detection results to send over
    for detection_object in detection_result.detections:
        bounding_box = detection_object.bounding_box
        (x,y,w,h) = bounding_box.origin_x, bounding_box.origin_y, bounding_box.width, bounding_box.height
        object_class = detection_object.categories[0].category_name
        results_string = b''
        if object_class in {'remote','cell phone','frisbee','cup','chair','vase'}:
            # ser.write(b"x,y,w,h,d,real_x,real_y\r\n");

            #results_string += struct.pack('f',float(max(x,0)))
            #results_string += struct.pack('f',float(max(y,0)))
            #results_string += struct.pack('f',float(w))
            #results_string += struct.pack('f',float(h))

            # Retrieve estimated relative distance, x, y
            distance = distanceSampler.getDistance(h)
            angle = distanceSampler.getAngle(x,y,w,h,width,height)
            angle_degrees = math.degrees(angle)
            (real_x_dist,real_y_dist) = distanceSampler.getRealXYFlatPlane(angle,distance)
            results_string += struct.pack('f',real_x_dist)
            results_string += struct.pack('f',real_y_dist)
            # results_string += struct.pack('f',distance)
            ser.write(results_string)
            ser.write(b'\r\n')

            # console print for debugging
            print(f'x,y,w,h,object_class: {x,y,w,h,object_class}')
            print(f'distance & angle: {distance, angle_degrees}')
            print(f'Estimated real x and y distance: {real_x_dist, real_y_dist}')

    if visualize_image:

        # Draw keypoints and edges on input image
        image = utils.visualize(image, detection_result)

        # Calculate the FPS
        if counter % fps_avg_frame_count == 0:
          end_time = time.time()
          fps = fps_avg_frame_count / (end_time - start_time)
          start_time = time.time()

        # Show the FPS
        fps_text = 'FPS = {:.1f}'.format(fps)
        text_location = (left_margin, row_size)
        cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                    font_size, text_color, font_thickness)

        # Stop the program if the ESC key is pressed.
        if cv2.waitKey(1) == 27:
          break
        cv2.imshow('object_detector', image)

    if cv2.waitKey(1)==27:
        break

  ser.close()
  cap.release()
  cv2.destroyAllWindows()

def main():


  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--model',help='Path of the object detection model.',required=False,default='efficientdet_lite0.tflite')
  parser.add_argument('--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument('--frameWidth',help='Width of frame to capture from camera.',required=False,type=int,default=640)
  parser.add_argument('--frameHeight',help='Height of frame to capture from camera.',required=False,type=int,default=480)
  parser.add_argument('--numThreads',help='Number of CPU threads to run the model.',required=False,type=int,default=4)
  parser.add_argument('--enableEdgeTPU',help='Whether to run the model on EdgeTPU.',action='store_true',required=False,default=False)
  parser.add_argument('--visualize',help='Show camera',action='store_true',required=False,default=False)
  args = parser.parse_args()
  run(args.model,args.cameraId,args.frameWidth,args.frameHeight,args.numThreads,args.enableEdgeTPU,args.visualize)

if __name__ == '__main__':
    import threading
    #x = threading.Thread(target=thread_test)
    #x.start()
    detect_thread = threading.Thread(target=main)
    detect_thread.start()
