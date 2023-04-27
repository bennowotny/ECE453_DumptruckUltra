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

from distance_sampler import DistanceSampler2


def run(
    model: str,
    camera_id: int,
    width: int,
    height: int,
    num_threads: int,
    enable_edgetpu: bool,
    visualize_image: bool
) -> None:
  
  ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
  ser.reset_input_buffer()    # flush

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

  pixel_size = 398.0      #3.98 um
  focalLength = 3.67     # in mm
  knownDistance = 50      # in mm
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
    # print(f'Input tensor shape {input_tensor.getHeight}')

    # Run object detection estimation using the model.
    detection_result = detector.detect(input_tensor)
    
    # Check detection results to send over
    for detection_object in detection_result.detections:
        bounding_box = detection_object.bounding_box
        (x,y,w,h) = bounding_box.origin_x, bounding_box.origin_y, bounding_box.width, bounding_box.height
        object_class = detection_object.categories[0].category_name
        #print(f'x,y,w,h,object_class: {x,y,w,h,object_class}')
        ser.write(b'{object_class},{x},{y},{w},{h}')
        ser.write(b'x,y,w,h,d,object_class: ')
        ser.write(max(x,0))     # bbox W can be negative
        ser.write(max(y,0))     # bbox H can be negative
        ser.write(w)
        ser.write(h)
        distance = distanceSampler.getDistance(y)
        ser.write(distance)
        ser.write(object_class.encode('utf-8'))
        ser.write(b'\n')
        # line = ser.readline().decode('utf-8').rstrip()

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
  parser.add_argument('--visualize',help='Show camera',action='store_true',required=False,default=True)
  args = parser.parse_args()
  
  run(args.model,args.cameraId,args.frameWidth,args.frameHeight,args.numThreads,args.enableEdgeTPU,args.visualize)

if __name__ == '__main__':
    main()
