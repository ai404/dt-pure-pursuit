#!/usr/bin/env python
import math
import time
import numpy as np
import rospy
import tf
from duckietown_msgs.msg import Twist2DStamped, LanePose#, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
from duckietown_msgs.msg import Segment, SegmentList

import time
import numpy as np
from collections import defaultdict

class lane_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.omega = 2
        self.pos_orientation = [0, 0]
        self.safety_start = True
        self.follow_point = None
        self.veh_name = self.setupParameter("~veh_name","default")

        self.beta = 0.9
        self.omega = 0
        # Publication
        self.pub_car_cmd = rospy.Publisher("/%s/joy_mapper_node/car_cmd"%self.veh_name, Twist2DStamped, queue_size=1)
        
        # Subscriptions
        self.sub_filtered_seg_list = rospy.Subscriber("/%s/lane_filter_node/seglist_filtered"%self.veh_name, SegmentList, self.update_pose)
        #self.sub_lane_pose = rospy.Subscriber("/%s/lane_filter_node/lane_pose"%self.veh_name, LanePose, self.update_orientation,queue_size=1)
        
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def loop_around(self, omega = -1):
        self.do(0, omega)

    @staticmethod
    def point2coords(point):
        return point.x, point.y, point.z

    @staticmethod
    def calculate_median(coords):
        return np.median(coords["x"]),np.median(coords["y"])
        #return sum(coords["x"])*1./n, sum(coords["y"])*1./n
        
    def process_segments(self, segments, offset,name=""):
        x_curve, y_curve = self.calculate_median(segments)

        x = x_curve 
        y = y_curve + offset

        self.follow_point = (x, y)
        rospy.loginfo("[%s] %s: Curve Point: %s"%(self.node_name, name, (x_curve, y_curve)))
    
    def update_pose(self, seg_list_msg):

        segments = defaultdict(lambda : defaultdict(list))

        for seg in seg_list_msg.segments:
            x1, y1, _ = self.point2coords(seg.points[0])
            x2, y2, _ = self.point2coords(seg.points[1])
            segments[seg.color]["x"].append(x1*.5+x2*.5)
            #segments[seg.color]["x"].append(x2)
            segments[seg.color]["y"].append(y1*.5+y2*.5)
            #segments[seg.color]["y"].append(y2)
        #rospy.loginfo("[%s] Points: %s"%(self.node_name, segments))

        
        # 0: white 1: yellow
        if len(segments.keys())>0:
            if self.safety_start:
                if len(segments[1]["x"])>0:
                    self.safety_start = False
                    # use yellow points
                    self.process_segments(segments[1] , offset=-0.2, name="yellow")
                else:
                    self.loop_around()
                """
                elif len(segments.keys()) >= 2:
                        x_white, y_white = self.calculate_mean(segments[0])
                        x_yellow, y_yellow = self.calculate_mean(segments[1])
                        self.follow_point = (x_white + x_yellow)*.5, (y_white + y_yellow)*.5
                """
            elif len(segments[1]["x"])>0:
                # use yellow points
                self.process_segments(segments[1] , offset=-0.1, name="yellow")
            elif len(segments[0]["x"])>0:
                # use white points
                #self.process_segments(segments[0] , offset=0.45, name="white")#without scaling
                self.process_segments(segments[0] , offset=0.3, name="white")# scaling omega with vnorm
            else:
                self.loop_around()
        else:
            self.loop_around()
        rospy.loginfo("[%s] Point to follow: %s"%(self.node_name, self.follow_point))

         
        #rospy.loginfo("[%s] colors found: %s"%(self.node_name, segments.keys()))
        v_init = .25
        self.gain = 4
        if self.follow_point is not None:
            #xs, ys = self.smooth_follow_point# = (1-self.beta)*self.smooth_follow_point + self.beta*self.follow_point
            # calculate velocity
            xf, yf = self.follow_point
            #xf, yf = (1 - self.beta)*xs +self.beta*xf, (1 - self.beta)*ys +self.beta*yf
            #self.smooth_follow_point = xf, yf
            vf_norm = math.sqrt( xf**2 + yf**2 )
            sin_theta = yf / vf_norm

            # negative right, positive left
            #self.omega = (1 - self.beta)*self.omega + self.beta*2*v_init*sin_theta*self.gain


            ### without scaling
            v = v_init#*vf_norm
            self.omega = sin_theta*self.gain

            ### with scaling
            #v = v_init*vf_norm*1.2
            #self.omega = 2*v_init*sin_theta/vf_norm*0.5

            self.do(v, self.omega)
            rospy.loginfo("[%s] Omega: %s"%(self.node_name, self.omega))
            #rospy.loginfo("[%s] Point to follow: %s"%(self.node_name, follow_point))
        

    def do(self,v,omega):
        car_control_msg = Twist2DStamped()
        car_control_msg.v = v
        car_control_msg.omega = omega
        self.pub_car_cmd.publish(car_control_msg)        
    
    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        
        # Send stop command
        self.do(0, 0)

        rospy.sleep(3)    #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)



if __name__ == "__main__":

    rospy.init_node("lane_controller_node", anonymous=False)  # adapted to sonjas default file

    lane_control_node = lane_controller()
    rospy.spin()
    #rospy.on_shutdown(custom_shutdown)
