#!/usr/bin/env python
import math
import time
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, SegmentList

import numpy as np
from collections import defaultdict

class lane_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()

        # This paramter is set inorder to make the robot turn around until it detect a yellow line
        self.safety_start = True
        
        self.veh_name = self.setupParameter("~veh_name","default")

        self.follow_point = None
        
        # Exponential smoothing parameters for the follow point
        self.beta = 0.9
        self.smooth_follow_point = None

        # initial velocity values
        self.omega = 0
        self.v_init = .3

        # gain for omega
        self.gain = 6

        # Publication
        #self.pub_car_cmd = rospy.Publisher("/%s/joy_mapper_node/car_cmd"%self.veh_name, Twist2DStamped, queue_size=1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        
        # Subscriptions
        #self.sub_filtered_seg_list = rospy.Subscriber("/%s/lane_filter_node/seglist_filtered"%self.veh_name, SegmentList, self.update_pose)
        #self.sub_lane_pose = rospy.Subscriber("/%s/lane_filter_node/lane_pose"%self.veh_name, LanePose, self.get_lane_info,queue_size=1)
        self.sub_filtered_seg_list = rospy.Subscriber("~seglist_filtered", SegmentList, self.update_pose)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.get_lane_info,queue_size=1)
        
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

        # init logs
        self.logging = True
        self.init_logs()

    def init_logs(self):
        if self.logging:
            with open("/tmp/logs-errors.csv","w") as logs_errors:
                logs_errors.write("Time,Heading error,Cross track error\n")
            
            with open("/tmp/logs-control.csv","w") as logs_control:
                logs_control.write("Time,Velocity,Omega\n")

    def add2logControl(self,v, omega):
        if self.logging:
            with open("/tmp/logs-control.csv","a") as logs_control:
                logs_control.write("%f,%f,%f\n"%(time.time(), v, omega))

    def add2logError(self, cross_track, heading):
        if self.logging:
            with open("/tmp/logs-errors.csv","a") as logs_errors:
                logs_errors.write("%f,%f,%f\n"%(time.time(), cross_track, heading))

    def get_lane_info(self, pose_msg):
        cross_track_err = pose_msg.d
        heading_err = pose_msg.phi

        self.add2logError(cross_track_err, heading_err)
        # rospy.loginfo("[%s] Cross track error: %s"%(self.node_name, cross_track_err))
        # rospy.loginfo("[%s] Heading error: %s"%(self.node_name, heading_err))

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
        
    def process_segments(self, segments, offset,name=""):
        x_curve, y_curve = self.calculate_median(segments)

        x = x_curve 
        y = y_curve + offset

        self.follow_point = (x, y)
        # rospy.loginfo("[%s] %s: Curve Point: %s"%(self.node_name, name, (x_curve, y_curve)))
    
    def update_pose(self, seg_list_msg):

        segments = defaultdict(lambda : defaultdict(list))
        
        # collect segment points and average the start and the end of a segment
        for seg in seg_list_msg.segments:
            x1, y1, _ = self.point2coords(seg.points[0])
            x2, y2, _ = self.point2coords(seg.points[1])
            segments[seg.color]["x"].append(x1*.5+x2*.5)
            segments[seg.color]["y"].append(y1*.5+y2*.5)
        
        # 0: white 1: yellow
        if len(segments.keys())>0:
            if self.safety_start:
                # at start the robot needs to spot a yellow lane first, 
                # otherwise it keeps looping until it finds one
                if len(segments[1]["x"])>0:
                    self.safety_start = False
                    # use yellow points
                    self.process_segments(segments[1] , offset=-0.2, name="yellow")
                else:
                    self.loop_around()
                """
                # we can check for both color but I think this approach is not necessary for LF
                elif len(segments.keys()) >= 2:
                        x_white, y_white = self.calculate_median(segments[0])
                        x_yellow, y_yellow = self.calculate_median(segments[1])
                        self.follow_point = (x_white + x_yellow)*.5, (y_white + y_yellow)*.5
                """
            elif len(segments[1]["x"])>0:
                # use yellow points
                self.process_segments(segments[1] , offset=-0.2, name="yellow")
            elif len(segments[0]["x"])>0:
                # use white points
                self.process_segments(segments[0] , offset=0.2, name="white")
            else:
                # if no lane is found loop around
                self.loop_around()
        else:
            self.loop_around()
        #rospy.loginfo("[%s] Point to follow: %s"%(self.node_name, self.follow_point))

        if self.follow_point is not None:
            xf, yf = self.follow_point

            # we can also use smoothing on the follow point but requires more tunning
            """
            if self.smooth_follow_point is not None:
                xs, ys = self.smooth_follow_point
                xf, yf = (1 - self.beta)*xs +self.beta*xf, (1 - self.beta)*ys +self.beta*yf
            self.smooth_follow_point = xf, yf
            """
            vf_norm = math.sqrt( xf**2 + yf**2 )
            sin_theta = yf / vf_norm

            ### without scaling
            # another interesting idea is to add a speed control 
            # based on the distance to the follow point
            v = self.v_init 
            self.omega = sin_theta*self.gain # negative right, positive left

            ### with scaling
            #v = self.v_init*vf_norm*1.2
            #self.omega = 2*self.v_init*sin_theta/vf_norm*0.5

            self.do(v, self.omega)
            #rospy.loginfo("[%s] Omega: %s"%(self.node_name, self.omega))
        

    def do(self,v,omega):
        if self.logging:
            self.add2logControl(v, omega)

        car_control_msg = Twist2DStamped()
        car_control_msg.v = v
        car_control_msg.omega = omega
        self.pub_car_cmd.publish(car_control_msg)        
    
    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_filtered_seg_list.unregister()
        self.sub_lane_pose.unregister()

        # Send stop command
        self.do(0, 0)

        rospy.sleep(1) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)



if __name__ == "__main__":

    rospy.init_node("lane_controller_node", anonymous=False)

    lane_control_node = lane_controller()
    rospy.spin()
