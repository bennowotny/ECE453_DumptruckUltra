# Base code adopted from https://github.com/tensorflow/examples/blob/master/lite/examples/object_detection/raspberry_pi/detect.py

import argparse
import sys
import time
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

import numpy as np
from tflite_support.task import processor

_MARGIN = 10    # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0,0,255)


def visualize(
    image: np.ndarray,
    output: processor.DetectionResult
) -> np.ndarray:
    '''
    Draws boundary boxes on input image
    Args:
        image: Input image
        output: Detection output
    Returns:
        Image with boundary boxes
    '''
    for detection in output.detections:

        '''
        # Draw boundary box
        bbox = detection.bounding_box
        start_point = bbox.origin_x, bbox.origin_y
        end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
        cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)

        # Draw label, score
        category = detection.categories[0]
        category_name = category.category_name
        probability = round(category.score,2)
        result_txt = f'{category_name}: {probability}'
        text_location = (_MARGIN + bbox.origin_x, _MARGIN + _ROW_SIZE + bbox.origin_y)
        cv2.putText(image, result_txt, text_location, cv2.FONT_HERSHEY_SIMPLEX, _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
        '''


        # Get bounding box coordinates
        ymin = int(max(1,(detection.bounding_box[0] * image.shape[0])))
        xmin = int(max(1,(detection.bounding_box[1] * image.shape[1])))
        ymax = int(min(image.shape[0],(detection.bounding_box[2] * image.shape[0])))
        xmax = int(min(image.shape[1],(detection.bounding_box[3] * image.shape[1])))

        # Draw bounding box
        cv2.rectangle(image,(xmin,ymin),(xmax,ymax),(10,255,0),2)

        # Draw label
        label = '{}: {:.0f}%'.format(detection.label_id,int(detection.score*100))
        label_size, base_line = cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX,_FONT_SIZE,_FONT_THICKNESS)
        top = max(ymin,label_size[1] + _MARGIN)
        cv2.rectangle(image,(xmin,top-label_size[1]-_MARGIN),(xmin+label_size[0],top+_MARGIN),(255,255,255),cv2.FILLED)
        cv2.putText(image,label,(xmin,top),cv2.FONT_HERSHEY_SIMPLEX,_FONT_SIZE,_TEXT_COLOR,_FONT_THICKNESS)
    return image



def run(
    model: str,
    cameraId: int,
    frameWidth: int,
    frameHeight: int,
    numThreads: int,
    enableEdgeTPU: bool,
    score_threshold: 0.3,
    iou_threshold: 0.5
) -> None:
    '''
    Run inference on the given model
    Args:
        model: Name of TFLite detection model
        cameraId: Camera ID
        frameWidth: Camera frame width
        frameHeight: Camera frame height
        numThreads: Number of CPU threads
        enableEdgeTPU: Enable Edge TPU
    '''
    counter, fps = 0,0
    start_time = time.time()

    cap = cv2.VideoCapture(cameraId)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,frameWidth)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,frameHeight)

    # Visualization parameters
    row_size = 20       # pixels
    left_margin = 24    # pixels
    text_color = (0,0,255)
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10

    # Model initialization
    base_options = core.BaseOptions(file_name=model,use_coral=enable_edgetpu,num_threads=numThreads)
    detection_options = processor.DetectionOptions(max_results=3,score_threshold=score_threshold,iou_threshold=iou_threshold)
    options = vision.ObjectDetectorOptions(base_options,detection_options)
    model = vision.ObjectDetector.create_from_options(options)

    # Inference loop
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            sys.exit('Failed to capture frame from camera.')
        counter += 1
        image = cv2.flip(frame,1)

        # Convert from BGR to RGB for TFLite model
        rgb_img = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        inputs = vision.TensorImage.create_from_array(rgb_img)
        output = model.detect(inputs)

        # Draw keypoints and edges on input image
        image = utils.visualize(image,output)

        # Compute fps
        if counter % fps_avg_frame_count == 0:
            fps = fps_avg_frame_count / (time.time() - start_time)
            start_time = time.time()
        
        # Draw fps on input image
        # image = utils.draw_fps(image,fps)
        fps_text = 'FPS: {:.1f}'.format(fps)
        cv2.putText(image,fps_text,(left_margin,row_size),cv2.FONT_HERSHEY_SIMPLEX,font_size,text_color,font_thickness)

        # Stop if ESC key is pressed
        if cv2.waitKey(1) == 27:
            break
        cv2.imshow('Object Detection',image)
    
    cap.release()
    cv2.destroyAllWindows()


def main():
    # TODO: Add additional command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',help='Path of detection model',required=False,default='efficientdet_lite0.tflite')
    parser.add_argument('--cameraId',help='Camera ID',required=False,type=int,default=0)
    parser.add_argument('--frameWidth',help='Camera frame width',required=False,type=int,default=640)
    parser.add_argument('--frameHeight',help='Camera frame height',required=False,type=int,default=480)
    parser.add_argument('--numThreads',help='Number of CPU threads',required=False,type=int,default=4)
    parser.add_argument('--enableEdgeTPU',help='Enable Edge TPU',required=False,type=bool,default=False)
    # Additional Args
    parser.add_argument('--threshold',help='Detection threshold',required=False,type=float,default=0.5)
    parser.add_argument('--iouThreshold',help='IOU threshold',required=False,type=float,default=0.5)
    args = parser.parse_args()

    run(args.model,args.cameraId,args.frameWidth,args.frameHeight,args.numThreads,args.enableEdgeTPU)


if __name__ == '__main__':
    main()