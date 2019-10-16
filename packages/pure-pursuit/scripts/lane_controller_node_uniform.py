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
from collections import defaultdict, deque

class lane_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.omega = 2
        self.pos_orientation = [0, 0]
        self.safety_start = True
        self.follow_point = None
        self.veh_name = self.setupParameter("~veh_name","default")

        self.beta = 0.95
        self.omega = 0
        # Publication
        self.pub_car_cmd = rospy.Publisher("/%s/joy_mapper_node/car_cmd"%self.veh_name, Twist2DStamped, queue_size=1)
        
        # Subscriptions
        self.sub_filtered_seg_list = rospy.Subscriber("/%s/lane_filter_node/seglist_filtered"%self.veh_name, SegmentList, self.update_segments)
        #self.sub_lane_pose = rospy.Subscriber("/%s/lane_filter_node/lane_pose"%self.veh_name, LanePose, self.update_orientation,queue_size=1)
        
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

        self.buffer_size = 10
        self.wQueue = deque(maxlen=self.buffer_size)
        self.yQueue = deque(maxlen=self.buffer_size)
        self.followPoints = deque(maxlen=5)
        self.wOffset = 0.2
        self.yOffset = -0.1
        self.update_interval = 0.1
        #self.main_loop()
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.update_interval), self.main_loop)
    
    def update_segments(self, seg_list_msg):
        rospy.loginfo("[%s] Updating Segments:"%self.node_name)
        for seg in seg_list_msg.segments:
            x1, y1, _ = self.point2coords(seg.points[0])
            x2, y2, _ = self.point2coords(seg.points[1])
            rospy.loginfo("[%s] Points: %s , %s"%(self.node_name, x1, y1))
            if seg.color == 0:
                self.wQueue.append([x1*.5+x2*.5,y1*.5+y2*.5])
            elif seg.color == 1:
                self.yQueue.append([x1*.5+x2*.5,y1*.5+y2*.5])

    def process_segments(self):
        x_med, y_med, n = 0, 0, 0
        if len(self.yQueue)>0:
            xs, ys = zip(*self.yQueue)
            x_med+= np.mean(xs)
            y_med+= np.mean(ys) + self.yOffset
            n+=1
            self.follow_point = (x_med, y_med)
        elif len(self.wQueue)>0:
            xs, ys = zip(*self.wQueue)
            #if np.mean(ys)>0:
                #self.loop_around(omega=-1)
            #else:
            x_med+= np.mean(xs)
            y_med+= np.mean(ys) + self.wOffset
            n+=1
            self.follow_point = (x_med, y_med)
        else:
            self.loop_around(omega=-1)
        #if n>0:
        #    self.followPoints.append(np.array([x_med/n,y_med/n]))
    
    def smoothen_points(self):
        if len(self.followPoints)>0:
            self.follow_point = self.followPoints[0]
            for point in [k for k in self.followPoints][1:]:
                self.follow_point = (1-self.beta)*self.follow_point + self.beta*point
    
    def main_loop(self,arg):
        rospy.loginfo("[%s] Start main loop: "%(self.node_name))
        self.process_segments()
        #self.smoothen_points()

        v_init = .2
        self.gain = 1
        rospy.loginfo("[%s] Follow Point: %s"%(self.node_name, self.follow_point))
        if self.follow_point is not None:
            # calculate velocity
            xf, yf = self.follow_point
            vf_norm = math.sqrt( xf**2 + yf**2 )
            sin_theta = yf / vf_norm

            ### without scaling
            v = v_init#*vf_norm
            self.omega = 2*v_init*sin_theta/vf_norm*self.gain

            self.do(v, self.omega)
            rospy.loginfo("[%s] Omega: %s"%(self.node_name, self.omega))
        
        #time.sleep(self.update_interval)

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def loop_around(self, omega = 2):
        self.do(0, omega)

    @staticmethod
    def point2coords(point):
        return point.x, point.y, point.z

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

        rospy.sleep(0.5)    #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)



if __name__ == "__main__":

    rospy.init_node("lane_controller_node", anonymous=False)  # adapted to sonjas default file

    lane_control_node = lane_controller()
    rospy.spin()
