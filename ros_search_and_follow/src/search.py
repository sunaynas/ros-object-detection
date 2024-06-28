
from ros_locate_msgs import locationMsg
from unitree_legged_msgs import HighCmdService
from enum import Enum


class SearchAndFollow:
    class State:
        SEARCHING = 'searching'
        FOLLOWING = 'following'

    def __init__(self, tolerance):
        rospy.init_node('search_algo', anonymous=True)
        self.rgb_data = rospy.Subscriber(
            "/grounded_dino_object_location/located_object_xoff",
            locationMsg,
            self.detection_callback
        )
        self.controller = rospy.ServiceProxy('/controller_node/control_a1', unitree_legged_msgs.srv.HighCmdService)
        self.locData = None
        self.tolerance = tolerance
        self.kp = 0.1
        self.state= self.state.SEARCHING
        

    def detection_callback(self, loc):
        self.locData = loc

    def search(self):
        p=0.
        if self.locData.found == False:
            p = 0.1 # or something like that 
        if self.locData.xoff < -self.tolerance:
            p = self.locData.xoff*self.kp
        else if self.locData.xoff > -self.tolerance:
            p = -self.locData.xoff*self.kp 
        else:
            self.state = self.state.FOLLOW
        self.controller(mode=2, forwardSpeed=0.0, sideSpeed=0.0, rotateSpeed=p)
    
    def follow(self):
        if self.locData.found == False:
            self.state = self.state.SEARCHING
            return
        p = 0.0
        if self.locData.xoff < -self.tolerance:
            p = self.locData.xoff*self.kp
        else if self.locData.xoff > -self.tolerance:
            p = -self.locData.xoff*self.kp 
        
       
        # can make pid to get closer to obj
        # else if self.locData.dist < 
        self.controller(mode=2, forwardSpeed=0.1, sideSpeed=0.0, rotateSpeed=p)
    
    def run (self):
        rate = rospy.Rate(50)  
        while not rospy.is_shutdown():
            if self.state == self.State.SEARCHING:
                search()
            else if self.state == self.State.FOLLOWING:
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
