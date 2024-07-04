#!/usr/bin/env python

import argparse

from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from  ros_locate_msgs import ObjLocation

import torch
import numpy as np
import PIL as pil
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

class ObjectLocation:
    def __init__(self, search_prompt):
        rospy.init_node('grounded_dino_object_location', anonymous=True)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_id = "IDEA-Research/grounding-dino-tiny"
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(self.device)
        self.cv_bridge = CvBridge()
        self.search_prompt = search_prompt
        self.rgb_data = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.rgb_callback
        )
        self.depth_data = rospy.Subscriber(
            "/camera/depth/image_rect_raw",
            Image,
            self.d_callback
        )
        self.latest_rgb_img = None
        self.latest_d_img = None
        self.publisher = rospy.Publisher('/located_object', ObjLocation, queue_size=10)

    def rgb_callback(self, rgb):
        self.latest_rgb_img = rgb
        self.find_obj_loc()

    def d_callback(self, d):
        self.latest_d_img = d
        self.find_obj_loc()

    def find_obj_loc(self):
        if self.latest_d_img is not None and self.latest_rgb_img is not None:
            #get the bounding box of the object
            obj_bbox = self.detect_object() 
            #use the bounding box to get the location of the object relative to the robot
            self.get_location(obj_bbox)
            #publish the location of the object
            self.latest_rgb_img = None
            self.latest_d_img = None 
    
    def detect_object(self):
        #convert ros image to pil image
        rgb_img = np.array(self.cv_bridge.imgmsg_to_cv2(self.latest_rgb_img, "rgb8"))
        rgb_img = pil.Image.fromarray(rgb_img)

        # perform Grounded Dino inference
        inputs = self.processor(images=rgb_img, text=self.search_prompt, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)
        
        # parse predictions
        results = self.processor.post_process_grounded_object_detection(
                outputs,
                inputs.input_ids,
                box_threshold=0.2, 
                text_threshold=0.2, 
                target_sizes=[rgb_img.size[::-1]]
            )[0]
        
        # find the best bounding box
        bestbbox = None
        highest_confidence = float(-1)
        for score, box in zip(results["scores"], results["boxes"]):
            box = [round(i, 2) for i in box.tolist()]
            confidence = float(score)
            if confidence > highest_confidence:
                highest_confidence = confidence
                bestbbox = box
        
        # return the bounding box of the object
        return bestbbox
        
    def get_location(self, bbox):
        ret = ob
        if bbox is None:
            self.obj_xoff_publisher.publish(found=false, xoff=0, dist =0)
            return
        depth_image = self.cv_bridge.imgmsg_to_cv2(self.latest_d_img, desired_encoding='passthrough')
        roi = depth_image[bbox[0]:bbox[2], bbox[1]:bbox[3]] #i think, but it might be swapped
        valid_depths = roi[np.isfinite(roi)]
        avg_depth = valid_depths.mean()
        # avg_depth^2 = x^2 + y^2 +z^2
        # but we know the angle 
        xcenter_roi = (bbox[0]+bbox[2])//2
        xcenter = depth_image.shape[1]//2
        self.obj_xoff_publisher.publish(found=true, xoff=xcenter_roi-xcenter, dist =avg_depth)
        return 

def main(args=None):
    parser = argparse.ArgumentParser(description="GroundedDino node")
    parser.add_argument(
        '--topic', 
        type=str, 
        default='chair', 
        help='What do you want to locate?'
    )
    parsed_args = parser.parse_args(args=args)
    
    obj_loc = ObjectLocation(search_prompt=parsed_args.topic) # move to args when finished debugging
    rospy.spin()

if __name__=="__main__":
    main()
