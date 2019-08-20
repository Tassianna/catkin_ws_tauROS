#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import inv
import math 
import random


class image_converter:

    def __init__(self):
       self.image_pub = rospy.Publisher("/sensors/camera/infra1/camera_info",Image,queue_size=1)
       self.image_pub_line=rospy.Publisher("/sensors/camera/infra1/line",Image,queue_size=1)
       self.bridge = CvBridge()
       self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw",Image,self.callback,queue_size=1)
   

    def callback(self,data):
        try:
           cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
           print(e)
        # to black white
        bi_max = 255
        bi_min = 240
        ret , thresh1 = cv2.threshold(cv_image, bi_min, bi_max, cv2.THRESH_BINARY)

        # crop
        cv2.rectangle(thresh1, (0, 0), (680, 30),0,-1)
        trapezoid = np.array( [(215,215),  (160,480),(600,480),(405,215)] )    
        triangle_1 = np.array( [(0,0), (210,0), (0,150)] )
        triangle_2 = np.array( [(680,0), (450,0), (680,120)] )
        cv2.drawContours(thresh1, [trapezoid], -1, 0, -1)
        cv2.drawContours(thresh1, [triangle_1], -1, 0, -1)
        cv2.drawContours(thresh1, [triangle_2], -1, 0, -1)

        def RANSAC(image,N,s,t):
            white=[]
            counter=-1
            for i in xrange(image.shape[0]):
                    for j in xrange(image.shape[1]):
                        if image[i, j] == 255:
                            #collect all the white points
                            white+=[[i,j]]
                            counter+=1

            max_line=[0,0,0,0,0]
            while N>0:
                #random sample 
                rp1 = random.randint(0,counter)
                rp2 = random.randint(0,counter)
                while rp1 == rp2:
                    rp2 = random.randint(0,counter)
                rp1 = white[rp1]
                rp2 = white[rp2]
                

                inliner=0
                if rp2[0]!=rp1[0]:
                    m=(rp2[1]-rp1[1])/(rp2[0]-rp1[0])
                    b=rp1[1]-m*rp1[0]
                else:
                    m=0
                    b=0
                    

                for points in white:
                    x=points[0]
                    y=points[1]
                    if rp2[0]!=rp1[0]:
                        dist=abs(b+m*x-y)/math.sqrt(1+m*m)
                    if rp2[0]==rp1[0]:
                        dist= 800
                    
                    if dist<t:
                        inliner+=1   

                if inliner > max_line[0]:
                    max_line=[inliner,m,b,rp1,rp2]
                N-=1
            return max_line

        i=3
        while i>0:
            _,m,b,rp1,rp2 = RANSAC(thresh1,150,2,10)
            p1=(rp1[1],rp1[0])
            p2=(rp2[1],rp2[0])
            print("m,b",m,b)
            cv2.line(thresh1,p1,p2,0,10,lineType=cv2.LINE_AA)
            cv2.line(cv_image,p1,p2,150,10,lineType=cv2.LINE_AA)
            i-=1
        print("--------------------------------------------------")

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
            self.image_pub_line.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
        except CvBridgeError as e:
             print(e)
   
def main(args):
        rospy.init_node('image_converter', anonymous=True)
        ic = image_converter()
       
        try:
            rospy.spin()
        except KeyboardInterrupt:
             print("Shutting down")
        cv2.destroyAllWindows()
    
if __name__ == '__main__':
         main(sys.argv)