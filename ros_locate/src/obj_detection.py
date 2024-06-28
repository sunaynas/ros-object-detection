#!/usr/bin/env python

"""
A node for performing object detection with grounded dino.
"""
import argparse
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import torch
import numpy as np
import PIL as pil
from PIL import ImageDraw, ImageFont
import requests

import cv2
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from ros_object_detection_msgs.msg import BoundingBox
# from ros_object_detection_msgs.srv import DetectObjectGroundedDino, DetectObjectGroundedDinoResponse


class GroundedDino:

    def __init__(self, image_topic: str):
        rospy.init_node('grounded_dino_object_detection', anonymous=True)
        self.logger = rospy.loginfo
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # pull image processor and detr model from huggingface
        model_id = "IDEA-Research/grounding-dino-tiny"
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(self.device)

        # create a bridge between ROS and OpenCV
        self.cv_bridge = CvBridge()

        # subscribe to camera image
        self.subscription = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self._image_callback,
            queue_size=1
        )

        # publish latest object detections (NOT IMAGE)
        self.detection_publisher = rospy.Publisher('/grounded_dino_detected_objects', Image, queue_size=10)
        
        # track latest image
        self._latest_rgb_image = None

        self.logger("Grounded Dino object detection node is ready.")

    def _image_callback(self, rgb):
        self._latest_rgb_image = rgb

    def _detect_object(self, request):
        # parse request
        text = request.prompt
        confidence_threshold = request.confidence_threshold

        # convert ROS image message to opencv
        rgb_img = np.array(self.cv_bridge.imgmsg_to_cv2(self._latest_rgb_image, "rgb8"))
        rgb_img = pil.Image.fromarray(rgb_img)

        # perform Grounded Dino inference
        self.logger('Running Grounded Dino Inference...')
        self.logger(f'Using Prompt {text}')
        inputs = self.processor(images=rgb_img, text=text, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        # parse predictions
        results = self.processor.post_process_grounded_object_detection(
                outputs,
                inputs.input_ids,
                box_threshold=0.2, # TODO: read this from config
                text_threshold=0.2, # TODO: read this from config
                target_sizes=[rgb_img.size[::-1]]
            )[0]

        # create bounding box response
        bboxs = []
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            box = [round(i, 2) for i in box.tolist()]
            # complete bounding box message
            bbox = BoundingBox()
            bbox.confidence = float(score)
            bbox.object = label.replace(' ', '_')
            bbox.xmin = int(box[0])
            bbox.ymin = int(box[1])
            bbox.xmax = int(box[2])
            bbox.ymax = int(box[3])

            bboxs.append(bbox)
        
        # this shld be diff
        # Draw bounding boxes on the image
        draw = ImageDraw.Draw(rgb_img)

        for bbox in bboxs:
            draw.rectangle([bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax], outline="red")
            # TODO: add label

        # Publish the annotated image
        annotated_img_msg = self.cv_bridge.cv2_to_imgmsg(np.array(rgb_img), encoding="rgb8")
        self.detection_publisher.publish(annotated_img_msg)        
        
        # return parsed predictions data
        response = DetectObjectGroundedDinoResponse()
        response.bounding_boxes = bboxs

        return response

def main(args=None):
    parser = argparse.ArgumentParser(description="GroundedDino node")
    parser.add_argument(
        '--topic', 
        type=str, 
        default='overhead_camera_rgb', 
        help='The topic name for the image feed'
    )
    parsed_args = parser.parse_args(args=args)
    
    grounded_dino = GroundedDino(image_topic=parsed_args.topic) # move to args when finished debugging
    rospy.spin()

if __name__=="__main__":
    main()
