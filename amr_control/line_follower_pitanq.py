#!/usr/bin/env python
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

####
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
        
        self.process_this_frame = self.droped_frames >= 1            

        if self.process_this_frame:
            #process_image_start_str = "process_image_start %s" % rospy.get_time()
            #rospy.loginfo(process_image_start_str)
            # We reset the counter
            #print("Process Frame, Dropped frame to==" + str(self.droped_frames))
            self.droped_frames = 0
            self.count=self.count + 1

            try:
                # We select bgr8 because its the OpenCV encoding by default
                cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
                cv_image = None

            if cv_image is not None:
                small_frame = cv2.resize(cv_image, (0, 0), fx=1 ,fy=1)               
                height, width, channels = small_frame.shape
                rospy.logdebug("height=%s, width=%s" % (str(height), str(width)))
                #rospy.loginfo("height=%s, width=%s" % (str(height), str(width)))
                crop_img = small_frame
                gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
                mask = cv2.GaussianBlur(gray,(5,5),0)  
                _, res= cv2.threshold(mask,100,255,cv2.THRESH_BINARY)  

                h=119
                x=0

                # slice image and calculate center of mass
                for y in range(0,480,120):
                    if y==0:
                     slice_img=res[y:y+h,x:x+width]
                     mid=ndimage.measurements.center_of_mass(255-slice_img)
                     sum_= 0.001*ndimage.sum(255-slice_img)
                     if str(mid[0]) != 'nan':
                        cv2.circle(slice_img, (int(mid[1]),int(mid[0])), 5, (255, 255, 255), -1)
                        cv2.putText(slice_img,"(Cx,Cy)=(%s,%s)" %(str(int(mid[1])),str(int(mid[0]))) , (5, 110), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0), 1,cv2.LINE_AA)
                        cv2.putText(slice_img,"sum=(%s)" %(str(sum_)) , (5, 90), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0), 1,cv2.LINE_AA)
                     else:
                         pass   
                        

                    elif y==120:
                     slice_img2=res[y:y+h,x:x+width]
                     mid2=ndimage.measurements.center_of_mass(255-slice_img2)
                     sum2_= 0.001*ndimage.sum(255-slice_img2)
                     if  str(mid2[0]) != 'nan':
                        cv2.circle(slice_img2, (int(mid2[1]),int(mid2[0])), 5, (255, 255, 255), -1)
                        cv2.putText(slice_img2,"(Cx,Cy)=(%s,%s)" %(str(int(mid2[1])),str(int(mid2[0]))) , (5, 110), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0), 1,cv2.LINE_AA)
                        cv2.putText(slice_img2,"sum=(%s)" %(str(sum2_)) , (5, 90), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0), 1,cv2.LINE_AA)
                     else:
                         pass

                    elif y==240:
                     slice_img3=res[y:y+h,x:x+width]  
                     mid3=ndimage.measurements.center_of_mass(255-slice_img3) 
                     sum3_= 0.001*ndimage.sum(255-slice_img3)
                     #rospy.loginfo("mid3[1]=%s, [0]=%s" % (str(mid3[1]), str(mid3[0])))
                     if  str(mid3[0]) != 'nan':
                        cv2.circle(slice_img3, (int(mid3[1]),int(mid3[0])), 5, (255, 255, 255), -1) 
                        cv2.putText(slice_img3,"(Cx,Cy)=(%s,%s)" %(str(int(mid3[1])),str(int(mid3[0]))) , (5, 110), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0), 1,cv2.LINE_AA)
                        cv2.putText(slice_img3,"sum=(%s)" %(str(sum3_)) , (5, 90), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0), 1,cv2.LINE_AA)
                     else:
                         pass

                    elif y==360:
                     slice_img4=res[y:y+h,x:x+width]   
                     mid4=ndimage.measurements.center_of_mass(255-slice_img4)
                     sum4_= 0.001*ndimage.sum(255-slice_img4)
                     if  str(mid4[0]) != 'nan':
                        cv2.circle(slice_img4, (int(mid4[1]),int(mid4[0])), 5, (255, 255, 255), -1) 
                        cv2.putText(slice_img4,"(Cx,Cy)=(%s,%s)" %(str(int(mid4[1])),str(int(mid4[0]))) , (5, 110), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0), 1,cv2.LINE_AA)
                        cv2.putText(slice_img4,"sum=(%s)" %(str(sum4_)) , (5, 90), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0), 1,cv2.LINE_AA)
                     else:
                         pass

                
                

                multi_img =np.vstack((slice_img,slice_img2,slice_img3,slice_img4))            
                cv2.imshow("multi_img",multi_img) 
                cv2.imshow("raw_img",small_frame)     
                cv2.waitKey(1)    
                slope=abs(mid4[1] - mid[1])
                deltaX= (mid3[1]-width/2.0)

                rospy.loginfo("deltaX=%s" % (str(deltaX)))
                


                
                self.error.append(deltaX)
                if(self.count==1):
                    self.time_amr.append(0)
                    self.begin_time=time.time()
                    self.stop=0
    
                    
                elif(self.count != 1):
                                   
                    high_ = abs(mid2[1]-mid3[1])+35
                    self.time_amr.append(time.time()-self.begin_time)
                    if (int(abs(mid[1]-mid2[1])) >int(high_)) : 
                         pass                   
                      #self.stop=1 
                 
                    else:
                         pass   
                    
                                            
                self.move_robot(height, width, slope, deltaX)
                                              
            else:
                pass
            
        else:
            self.droped_frames += 1
            #print("Droped Frames==" + str(self.droped_frames))
            
            
            
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
