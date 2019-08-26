#!/usr/bin/env python

import math
import numpy as np
import rospy
import sys
from scipy.interpolate import CubicSpline 
from geometry_msgs.msg import Point,PointStamped
from visualization_msgs.msg import Marker 
from std_msgs.msg import ColorRGBA

# max1 12,7 max2:14,7
# min 0

class Drive_spline:
    def __init__(self):
        self.lane1 = np.load("lane1.npy") 
        self.supports1 = self.lane1[[0, 100, 150, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1276], :]
        self.lane2 = np.load("lane2.npy") 
        self.supports2 = self.lane2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :]
          
        self.plot_pub = rospy.Publisher("visualization_msgs/Marker",Marker,queue_size=1)
        self.points_sub = rospy.Subscriber("/clicked_point",PointStamped,self.callback,queue_size=1)
        
    
    def marker_points(self,lane,supports):
            u_vec = np.array(supports[:,[0]]).flatten() 
            x_vec = np.array(supports[:,[1]]).flatten() 
            y_vec = np.array(supports[:,[2]]).flatten() 
            xspline = CubicSpline(u_vec, x_vec) #polynom u=ax^n+bx^n-1...
            yspline = CubicSpline(u_vec, y_vec)
            #choose  points every 1cm
            u_vector = np.array(lane[:,[0]]).flatten()
            choose_u=[u_vector[0]]
            for u in u_vector:
                if abs(u-choose_u[-1])>0.01:
                    choose_u+=[u]

            xs = xspline(choose_u)
            ys = yspline(choose_u)
            makerP = [Point(x,y,0) for x,y in zip(xs,ys) ]
            return makerP,xspline,yspline

    def make_Marker(slef,markerP,id):

            marker=Marker()
            marker.type = Marker.LINE_STRIP
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = id
            marker.action = marker.ADD

            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0
            marker.scale.z = 0
            marker.color.a = 1.0 
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0

            marker.points = markerP
            marker.colors = [ColorRGBA(1,0,0,1) for i in markerP]
            return marker
        
    def make_point_Marker(self,markerP,id,color):
        marker=Marker()
        marker.type = Marker.SPHERE
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = id
        marker.action = marker.ADD

        marker.pose.position = markerP
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color = color
        
        return marker
    
    def distance(self,cx,cy,x,y):
        return math.sqrt((cx-x)*(cx-x)+(cy-y)*(cy-y))

    def find_closest(self,cx,cy,xsp1,ysp1,max_u):
                s = int(max_u/4)
                choose_u = [s,2*s,3*s]
                u = 0
                xs1 = xsp1(u) 
                ys1 = ysp1(u)
                dist1 = self.distance(cx,cy,xs1,ys1)
                # to aviod local minimal
                for i in choose_u:
                    xs1 = xsp1(i) 
                    ys1 = ysp1(i)
                    dist = self.distance(cx,cy,xs1,ys1)
                    if dist<dist1:
                        u=i
                        dist1=dist

                #fix begin dirction
                dirc = 1
                step = 0.1
                new_u= u + dirc*step
                new_xs = xsp1(new_u) 
                new_ys = ysp1(new_u) 
                new_dist =  self.distance(cx,cy,new_xs,new_ys)
                
                if new_dist>dist1:
                    dirc = -1
                    new_u = u+dirc*step
                    if new_u<0:
                        new_u+=max_u
                    new_xs =xsp1(new_u) 
                    new_ys =ysp1(new_u)
                    new_dist =  self.distance(cx,cy,new_xs,new_ys) 
                
                # binary  search
                while step >0.001:
                    while new_dist<dist1:
                        dist1=new_dist
                        new_u= new_u+dirc*step
                        if new_u > max_u :
                            new_u -=max_u 
                        if new_u<0:
                            new_u+=max_u
                        new_xs =xsp1(new_u) 
                        new_ys =ysp1(new_u)
                        new_dist =  self.distance(cx,cy,new_xs,new_ys)
    
                    dirc = -dirc
                    step = step/2
                    dist1=new_dist
                    new_u= new_u+dirc*step
                    if new_u > max_u :
                        new_u -=max_u 
                    if new_u<0:
                        new_u+=max_u
                    new_xs =xsp1(new_u) 
                    new_ys =ysp1(new_u)
                    new_dist =  self.distance(cx,cy,new_xs,new_ys)
                  
                return Point(new_xs,new_ys,0)

    def find_look_ahead(self,cx,cy,xsp1,ysp1,max_u1):
        closeP = self.find_closest(cx,cy,xsp1,ysp1,max_u1)
        xp = closeP.x
        yp = closeP.y
        a = self.distance(cx,cy,xp,yp) 
        b = math.sqrt(9+a*a)
        cx1 = xp+3/b
        cy1 = yp+a/b 
        return self.find_closest(cx1,cy1,xsp1,ysp1,max_u1)

    def callback(self,data):
        _,xsp1,ysp1 = self.marker_points(self.lane1,self.supports1)
        _,xsp2,ysp2 = self.marker_points(self.lane2,self.supports2)
        max_u1 = self.lane1[[-1],[0]]
        max_u2 = self.lane2[[-1],[0]]
        click = data.point
        cx = click.x
        cy = click.y
        color_close = ColorRGBA(1,1,1,1)
        color_ahead = ColorRGBA(0,1,0,1)
        marker3 = self.make_point_Marker(self.find_closest(cx,cy,xsp1,ysp1,max_u1),3,color_close)
        marker4 = self.make_point_Marker(self.find_closest(cx,cy,xsp2,ysp2,max_u2),4,color_close)
        marker5 = self.make_point_Marker(self.find_look_ahead(cx,cy,xsp1,ysp1,max_u1),5,color_ahead)
        marker6 = self.make_point_Marker(self.find_look_ahead(cx,cy,xsp2,ysp2,max_u2),6,color_ahead)



        self.plot_pub.publish(marker3)
        self.plot_pub.publish(marker4)
        self.plot_pub.publish(marker5)
        self.plot_pub.publish(marker6)


        

    def pub_marker(self): 

        markerP1 = self.marker_points(self.lane1,self.supports1)[0]
        markerP2 = self.marker_points(self.lane2,self.supports2)[0]
        marker1 = self.make_Marker(markerP1,1)
        marker2 = self.make_Marker(markerP2,2)


        while not rospy.is_shutdown():
            self.plot_pub.publish(marker1)
            self.plot_pub.publish(marker2)
            rospy.sleep(1)
        


def main(args):
        rospy.init_node("Spline") 
        control = Drive_spline()
        control.pub_marker()

        try:
            rospy.spin()
        except:
            print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)