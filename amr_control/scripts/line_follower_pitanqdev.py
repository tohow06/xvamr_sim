#!/usr/bin/env python3
import rospy
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import threading
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from scipy import ndimage
from std_msgs.msg import Int8
from std_msgs.msg import Float32

#### Ref:   https://github.com/tprlab/pitanq-dev/tree/master/selfdrive/follow_line
from follow_line import track_cv as track

####


class LineFollower(object):
    def __init__(self, camera_topic="/usb_cam/image_raw", cmd_vel_topic="/cmd_vel",state_topic="/follow_state",cmb_topic="/follow_cmd",ultrasonic_topic="/amr_adjust"):


        # We check which OpenCV version is installed.
        (self.major, minor, _) = cv2.__version__.split(".")
        rospy.logwarn("OpenCV Version Installed==>"+str(self.major))

        # This way we process only half the frames
        self.process_this_frame = True
        self.droped_frames = 0
        # 1 LEFT, -1 Right, 0 NO TURN
        self.last_turn = 0
        self.count=0
        self.time_amr=[]       
        self.error=[]
        self.begin_time=0  
        self.stop=0
        self.follow_cmd=0 
        self.integral=0   
        self.bridge_object = CvBridge()

        self.image_sub = rospy.Subscriber(camera_topic, Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.amr_pub= rospy.Publisher(state_topic, Int8,  queue_size=1)
        self.amr_sub = rospy.Subscriber(cmb_topic, Int8, self.ipc_cmd, queue_size=1)
        self.amr_sub = rospy.Subscriber(ultrasonic_topic, Float32, self.amr_adjust, queue_size=1)
    


    def ipc_cmd(self, data):
        if (data.data==1):
            self.follow_cmd=1
        elif (data.data==2):
            self.follow_cmd=2
        elif (data.data==3):
            self.follow_cmd=3

    def amr_adjust(self, data):
        if ( self.follow_cmd==3 and data.data<10 ):
            cmd_vel = Twist()
            cmd_vel.linear.x = -0.05
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.logdebug("amr_adjust=%s, " % (str(data.data)))
        elif (self.follow_cmd==3 and data.data>20 ):
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.05
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.logdebug("amr_adjust=%s, " % (str(data.data)))
        elif (self.follow_cmd==3 and (data.data<20 or data.data>10) ):
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
        
    

    def camera_callback(self, data):
   
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #cv2.imshow("cv_image", cv_image)
        #cv2.waitKey(1)
        angle, shift = track.handle_pic(cv_image, show = False)
        #print("ttrack.handle_pic(cv_image) angle = ",angle,"shift = ",shift)
        height = width = slope = deltaX = 0
        self.move_robot(height, width, slope, deltaX)
                                              
            
            
    def move_robot(self, image_dim_y, image_dim_x, slope, deltaX, linear_vel_base = 0.05,  angular_vel_base = 0.1, movement_time = 0.05):

        #rospy.loginfo(str(image_dim_y))
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0

        max_angular=0.5
        max_linear=0.1
        max_error=320
        
        
        Kp_ANGULAR = max_angular / max_error
        Kp_LINEAR = max_linear / max_error
        Ki=0.001

        self.integral=0.5*self.integral+deltaX    

        delta_left_percentage_not_important = 0.0001

        
        if  deltaX is not None:
            origin = [image_dim_x / 2.0, image_dim_y / 2.0]
            delta_left_right= deltaX
            if abs(delta_left_right) <= image_dim_x * delta_left_percentage_not_important:
                delta_left_right = 0.0

            delta = [delta_left_right, slope]
            # -1 because when delta is positive we want to turn right, which means sending a negative angular
            cmd_vel.angular.z = delta[0] * Kp_ANGULAR * -1 + self.integral*Ki*-1          
            cmd_vel.linear.x =  -abs(delta[0])*Kp_LINEAR + max_linear

            if cmd_vel.angular.z > 0:
                self.last_turn = 1
            elif cmd_vel.angular.z < 0:
                self.last_turn = -1
            elif cmd_vel.angular.z == 0:
                self.last_turn = 0

            
        else:
            cmd_vel.linear.x = 0.0

            if self.last_turn > 0:
                cmd_vel.angular.z = -angular_vel_base
            elif self.last_turn <= 0:
                cmd_vel.angular.z = angular_vel_base
                print("NO CENTROID DETECTED...SEARCHING...")      


        if (self.follow_cmd==1 and self.stop==0):
            self.cmd_vel_pub.publish(cmd_vel)
            
        elif (self.stop == 1 and self.follow_cmd==1 ):
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            self.amr_pub.publish(1)
        elif (self.follow_cmd==2 and self.stop==1):
            cmd_vel.linear.x = -0.01
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            self.amr_pub.publish(2)    
       
        # We move for only a fraction of time
        init_time = rospy.get_time()
        finished_movement_time = False
        rate_object = rospy.Rate(20)
        while not finished_movement_time:
            now_time = rospy.get_time()
            delta = now_time - init_time
            finished_movement_time = delta >= movement_time
            rate_object.sleep()

    def loop(self):        
        def thread_job():
            while(1):               
              if(self.stop==0) :   
                  pass        
              elif (self.stop==1):
               #plt.plot(self.time_amr,self.error)
               #plt.show()
               sys.exit()
          
        add_thread = threading.Thread(target = thread_job)      
        add_thread.start()
        
        rospy.spin()
        
    

if __name__ == '__main__':
    
    rospy.init_node('line_follower_start', anonymous=True)       
    robot_mover = LineFollower()
    robot_mover.loop()
        
        
 #rostopic pub /follow_cmd std_msgs/Int8 1
