#!/usr/bin/env python

import math
import rospy
import sys
import tf 
from autominy_msgs.msg import NormalizedSteeringCommand,SpeedCommand
from nav_msgs.msg import Odometry

class PID_Controller:
    def __init__(self):
        self.pre_time = rospy.Time.now()
        self.yaw= 0
        self.setpoint= 0
        self.acc_error= 0
        self.pre_error= 0
        self.gps_sub = rospy.Subscriber("/communication/gps/15", Odometry, self.call_back)
        self.str_pub = rospy.Publisher("/actuators/steering_normalized",NormalizedSteeringCommand, queue_size=1)
        self.speed_pub = rospy.Publisher("/actuators/speed",SpeedCommand,queue_size=1)

    def call_back (self,raw_msgs):
                orientation= raw_msgs.pose.pose.orientation
                #math transformation
                quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                euler = tf.transformations.euler_from_quaternion(quaternion) 
                self.yaw = euler[2]
                self.PID()

    def PID(self):	
            Kp= 5
            Ki= 0
            Kd= 0.1
            error= (self.setpoint - self.yaw) 

            self.acc_error = self.acc_error + error #accumulator for error
            current_time = rospy.Time.now()
            delta_time = (current_time - self.pre_time).to_sec()
            #Kp*error Ki*area Kd*slope

            PID= Kp * error + Ki * self.acc_error * delta_time + Kd * (error - self.pre_error) / delta_time
            # PID = int(round(PID))
            self.pre_time = current_time
            self.pre_error = error

            #pid value to steering message
            str_com=NormalizedSteeringCommand()
            str_com.value=PID
            self.str_pub.publish(str_com)
            
            #pub speed
            sp=SpeedCommand()
            sp.value=0.3
            self.speed_pub.publish(sp)
           

def main(args):
        rospy.init_node("PID_Control") 
        control = PID_Controller() 
        try:
            rospy.spin()
        except:
            print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)