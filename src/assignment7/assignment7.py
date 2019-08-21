#!/usr/bin/env python

import math
import rospy
import sys
import tf 
from autominy_msgs.msg import NormalizedSteeringCommand,SpeedCommand
from nav_msgs.msg import Odometry

class PID_Controller:
    def __init__(self):
        self.last_time = rospy.Time.now()
        self.yaw= 0. 
        self.setpoint= 0
        self.acc_error= 0.
        self.last_error= 0.
        self.sub_odom = rospy.Subscriber("/communication/gps/15", Odometry, self.call_back)
        self.str_pub = rospy.Publisher("/actuators/steering_normalized",NormalizedSteeringCommand
, queue_size=1)
        self.speed_pub = rospy.Publisher("/actuators/speed",SpeedCommand,queue_size=1)


    def call_back (self,raw_msgs):
                orientation= raw_msgs.pose.pose.orientation
                #math transformation
                quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                euler = tf.transformations.euler_from_quaternion(quaternion) 
                self.yaw = euler[2]
                self.PID()

    def PID(self):	
            #pid konfiguartion
            Kp= 2
            Ki= 0.5
            Kd= -3
            error= (self.setpoint - self.yaw) 

            self.acc_error = self.acc_error + error
          
            current_time = rospy.Time.now()
            dif_time = (current_time - self.last_time).to_sec()
            #line of pid calculation kind of integral and diferential
            PID= Kp * error + Ki * self.acc_error * dif_time + Kd * (error - self.last_error) / dif_time
            PID = int(round(PID))
             
            self.last_time = current_time
            self.last_error = error

    `       #pid value to steering message
            str_com=NormalizedSteeringCommand()
            str_com.value=PID
            self.str_pub.publish(str_com)
            
            #pub speed
            sp=SpeedCommand()
            sp.value=0.3
            self.speed_pub.publish(sp)
           

def main(args):
        rospy.init_node("Local_GPS_data") # have to define node for ROS
        control = PID_Controller() # call the class
        try:
            rospy.spin()
        except:
            print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)