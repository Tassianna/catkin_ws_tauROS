 #!/usr/bin/env python

import math
import rospy
import sys
import numpy as np
import tf 
from autominy_msgs.msg import NormalizedSteeringCommand,SpeedCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import  map as myMap

class Navigation:
    def __init__(self):
        self.pre_time = rospy.Time.now()
        self.yaw= 0
        self.setpoint= 0
        self.acc_error= 0
        self.pre_error= 0
        self.lane = 0

        self.loca_sub = rospy.Subscriber("/sensors/localization/filtered_map",Odometry,self.callback,queue_size=10)
        self.lane_sub = rospy.Subscriber("/lane_change",Int32 , self.change_lane)
        self.str_pub = rospy.Publisher("/actuators/steering_normalized",NormalizedSteeringCommand, queue_size=10)
        self.speed_pub = rospy.Publisher("/actuators/speed",SpeedCommand,queue_size=10)
    
    def change_lane(self,raw_msgs):
        if raw_msgs.data == 0:
            self.lane = 0
        else:
            self.lane = 1
        
    def callback(self,raw_msgs):
        position= raw_msgs.pose.pose.position
        x = position.x
        y = position.y
        current_point = [x,y]
        # math transformation
        orientation= raw_msgs.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion) 
        self.yaw = euler[2]
        def rot(yaw):
            return np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        
        map = myMap.Map()
        lane = map.lanes[self.lane]
        goal_point = lane.lookahead_point(current_point,0.5)[0]
        vector = goal_point-current_point
        map_point = np.matmul(rot(-self.yaw), vector)
        self.setpoint = np.arctan2(map_point[1],map_point[0])
        #  np.arccos(np.dot(current_point,goal_point)/(np.linalg.norm(current_point)*np.linalg.norm(goal_point)))

        Kp= 2.0
        Ki= 0.0
        Kd= 0.2
        error= self.setpoint #- self.yaw

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
        str_com.value = PID
        self.str_pub.publish(str_com)
        
        #pub speed
        sp=SpeedCommand()
        sp.value=0.3
        self.speed_pub.publish(sp)

def main(args):
        rospy.init_node("Navigation_Control") 
        control = Navigation() 
        try:
            rospy.spin()
        except:
            print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
    