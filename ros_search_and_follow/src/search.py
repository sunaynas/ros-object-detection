#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image 
from geometry_msg.msg import Point
from unitree_legged_msgs import HighCmdService
from enum import Enum
from cv_bridge import CvBridge
import numpy as np


class SearchAndFollow:
    class State:
        SEARCHING = 'searching'
        FOLLOWING = 'following'

    def __init__(self, xtolerance, dtolerance):
        rospy.init_node('search_algo', anonymous=True)
        self.pt1_sub = rospy.Subscriber(
            "located_object_minPt",
            Point,
            self.min_pt_callback
        )
	self.pt2_sub = rospy.Subscriber(
            "located_object_maxPt",
            Point,
            self.max_pt_callback
        )
	self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_rect_raw",
            Image,
            self.depth_callback
        )
	
        self.controller = rospy.ServiceProxy(
	    '/controller_node/control_a1', 
	    unitree_legged_msgs.srv.HighCmdService
	)

	self.cv_bridge = CvBridge()

        self.xoff = Int32(0)
        self.obj_detected = False
	self.depth_map = None
	self.min_pt = None
	self.max_pt = None

        self.x_tolerance = abs(xtolerance)
	self.dist_tolerance = abs(dtolerance)
        self.kp = 0.1
        self.state= self.state.SEARCHING
        

    def min_pt_callback(self, min_pt):
        self.min_pt = min_pt

    def max_pt_callback(self, max_pt):
        self.max_pt = max_pt

    def process_detection(self):
	if self.min_pt is not None and self.max_pt is not None and self.depth_map is not None:
	   if self.min_pt.x <= 0:
		self.obj_detected = False
	   else:		
		self.xoff = (self.depth_map.shape[1])//2 - (max_pt.x + min_pt.x)//2
       		self.obj_detected = True
	   self.min_pt = None
	   self.max_pt = None


    def depth_callback(self, depth_image):
        self.depth_map = self.cv_bridge.imgmsg_to_cv2(depth_img, desired_encoding='passthrough')

    def search(self):
        p=0.0
        if self.obj_detected == False:
            p = 0.1 # or something like that 
        elif self.xoff < -self.x_tolerance:
            p = Float32(self.xoff)*self.kp
        elif self.xoff > self.x_tolerance:
            p = -Float32(self.xoff)*self.kp 
        else:
            self.state = self.state.FOLLOW
        self.controller(mode=2, forwardSpeed=0.0, sideSpeed=0.0, rotateSpeed=p)
    
    def follow(self):
        if self.obj_detected == False:
            self.state = self.state.SEARCHING
            return
        p = 0.0
        if self.xoff < -self.x_tolerance:
            p = min(Float32(self.xoff)*self.kp, 0.1)
        elif self.xoff > -self.x_tolerance:
            p = max(-Float32(self.xoff)*self.kp, -0.1) 
        
        obj_dist = np.mean(self.depth_image[self.min_pt.x:self.max_pt.x, self.min_pt.y:self.max_pt.y])
	closest = np.min(self.depth_image)
	
	fs = 0.0
	ss = 0.0
	if (closest > self.dist_tolerance):
	    fs = min(0.1, self.kp*obj_dist)
	else:
	    lft_dist = np.mean(self.depth_image[:, :width//2])
	    right_dist = np.mean(self.depth_image[:, width//2:])
	    if lft_dist > right_dist:
		ss = -0.05
	    else:
		ss = 0.05

        # can make pid to get closer to obj
        # else if self.locData.dist < 
        self.controller(mode=2, forwardSpeed=fs, sideSpeed=ss, rotateSpeed=p)
    
    def run (self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
	    process_detection()
            if self.state == self.State.SEARCHING:
                search()
            elif self.state == self.State.FOLLOWING:
                follow()
            rate.sleep()
    

def main(args=None):
    parser = argparse.ArgumentParser(description="Search and follow node")
    parser.add_argument(
        '--tolerance', 
        type=Int32, 
        default='10', 
        help='Location tolerance (in px)'
    )
    parsed_args = parser.parse_args(args=args)
    
    search = SearchAndFollow(tolerance=parsed_args.tolerance) # move to args when finished debugging
    search.run()
    rospy.spin()

if __name__=="__main__":
    main()
