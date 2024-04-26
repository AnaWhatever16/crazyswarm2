#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, PoseArray, Pose
from tf2_msgs.msg import TFMessage
import numpy as np
from crazyflie_interfaces.msg import FullStateArray, FullState

import time

from numpy import *
import matplotlib.pyplot as plt

amorti = True

Time = []
Func = []
x = []
y = []
dx = []
dy = []
Cx = []
Cy = []

R = 1.0
def is_pos_def(x):
    return np.all(np.linalg.eigvals(x) >= 0)

Rpi2 = np.array([[0,-1],[1,0]])

class Circle():
    N = 0
    def __init__(self, C, r):
        self.C = array(C, dtype=float)
        self.r = r
        self.points = []
        self.N = Circle.N
        Circle.N += 1
        self.gradient = zeros(2)
        self.edges = []
        self.isolated = False
        
    def clean(self):
        self.points = []
        self.gradient *= 0
        self.edges = []
        self.isolated = False
        
        
    def CC_intersection(self, circle):
        D = circle.C - self.C
        d = linalg.norm(D)
        if(d < self.r + circle.r):
            Ud = D/d; UQd = array([-Ud[1], Ud[0]])
            x = (self.r*self.r + d*d - circle.r*circle.r)/(2*d)
            y = sqrt(self.r*self.r - x*x)
            P1 = Point(self.C + Ud*x + UQd*y, circle, self)
            P2 = Point(self.C + Ud*x - UQd*y, self, circle)
            
            P1.subhessian(P1.P - P1.Cin.C, P1.P - P1.Cout.C)
            P2.subhessian(P2.P - P2.Cin.C, P2.P - P2.Cout.C)
            
            self.points.append(P1); self.points.append(P2)
            circle.points.append(P1); circle.points.append(P2)
            
    def CS_intersection(self, segment):
        A = segment.P1; B = segment.P2; C = self.C
        U = B-A; V = C-A
        u = linalg.norm(U)
        Eu = U/u
        y = cross(Eu,V)
        if(abs(y) < self.r):
            x = sqrt(self.r*self.r - y*y)
            s = V@Eu
            
            P1 = Point(A + (s-x)*Eu, segment, self)
            self.points.append(P1); segment.points.append(P1)
            P2 = Point(A + (s+x)*Eu, self, segment)
            self.points.append(P2); segment.points.append(P2)
            
            P1.subhessian(P1.Cin.normal, P1.P - P1.Cout.C)
            P2.subhessian(P2.P - P2.Cin.C, P2.Cout.normal)
    
    def filter_points(self):
        self.isolated = len(self.points) == 0
        self.points.sort(key = lambda x: arctan2(x.P[1] - self.C[1],x.P[0] - self.C[0]))
        for i in range(len(self.points)):
            if(self.points[i].Cin == self):
                for j in range(i+1, i+len(self.points)):
                    if(self.points[j%len(self.points)].Cin == self.points[i].Cout):
                        break
                    else:
                        self.points[j%len(self.points)].order += 1
        self.points = list(filter(lambda x: x.order == 0, self.points))

    def compute_gradient(self):
        if(len(self.points)==0):
            return zeros(2)
        gradient = zeros(2)
        if(self.points[0].Cin == self):
            self.points = self.points[1:] + self.points[0:1]
        for i in range(0, len(self.points),2):
            gradient -= self.points[i+1].P - self.points[i].P
        gradient = array([-gradient[1], gradient[0]])
        self.gradient = gradient
        return gradient
    
    def integrate(self, plot=False):
        self.edges = []
        for i in range(0, len(self.points),2):
            self.edges.append([self.points[i].P,self.points[i+1].P])
        integral = 0
        for edge in self.edges:
            theta1 = arctan2(edge[0][1] - self.C[1],edge[0][0] - self.C[0])
            theta2 = arctan2(edge[1][1] - self.C[1],edge[1][0] - self.C[0])
            if(theta1 > theta2):
                theta2 += 2*pi
            
            if(plot):
                theta = linspace(theta1, theta2, 100)
                x = self.C[0] + self.r*cos(theta)
                y = self.C[1] + self.r*sin(theta)
                plt.plot(x,y, 'b')
            # dx = diff(x)
            # dy = diff(y)
            # integral += (dx@y[1:] - dy@x[1:])/2
            integral -= self.r**2*(theta2-theta1)/2 + np.cross(self.C, edge[1] - edge[0])/2
        
        if(self.isolated):
            integral -= pi*self.r**2
        return integral

    def plot(self):
        theta = linspace(0,2*pi, 1000)
        plt.plot(self.C[0] + self.r*cos(theta), self.C[1] + self.r*sin(theta), 'k')
        plt.plot(self.C[0], self.C[1], 'ro')
        plt.text(self.C[0], self.C[1], str(self.N))
        plt.arrow(self.C[0], self.C[1], self.gradient[0], self.gradient[1])

class Segment():
    N = 0
    def __init__(self, P1, P2):
        self.P1 = array(P1)
        self.P2 = array(P2)
        self.points = []
        self.N = Segment.N
        Segment.N += 1
        self.normal = array([[0,-1],[1,0]])@(self.P2-self.P1)

    def clean(self):
        self.points = []
    
    def filter_points(self):
        self.points.append(Point(self.P1, None, self))
        self.points.append(Point(self.P2, self, None))
        self.points.sort(key = lambda x: (x.P-self.P1)@(self.P2-self.P1))
        for i in range(len(self.points)):
            s = (self.points[i].P - self.P1)@(self.P2-self.P1)/linalg.norm(self.P2-self.P1)**2
            if(self.points[i].Cin == self):
                for j in range(i+1, len(self.points)):
                    if(self.points[j].Cin == self.points[i].Cout):
                        break
                    else:
                        self.points[j].order += 1
        self.points = [x for x in filter(lambda x: x.order == 0, self.points)]

    def integrate(self, plot = False):
        integral = 0
        
        for i in range(1,len(self.points),2):
            AA = self.points[i-1].P
            BB = self.points[i].P
            if(plot):
                plt.plot([self.points[i-1].P[0], self.points[i].P[0]],[self.points[i-1].P[1], self.points[i].P[1]], 'b')
            # x = linspace(self.points[i-1].P[0], self.points[i].P[0], 1000)
            # y = linspace(self.points[i-1].P[1], self.points[i].P[1], 1000)
            # dx = diff(x)
            # dy = diff(y)
            # integral += (dx@y[1:] - dy@x[1:])/2
            # print((dx@y[1:] - dy@x[1:])/2, ' = ', (AA[1]*BB[0] - AA[0]*BB[1])/2)
            integral += (AA[1]*BB[0] - AA[0]*BB[1])/2
            
        return integral
    
    def plot(self):
        plt.plot([self.P1[0], self.P2[0]],[self.P1[1], self.P2[1]], 'r')
        #plt.text((self.P1[0] + self.P2[0])/2,(self.P1[1] + self.P2[1])/2,str(self.N))

class Point():
    N = 0
    def __init__(self, P, Cin, Cout):
        self.P = P
        self.Cin = Cin
        self.Cout = Cout
        self.h = zeros((2,2))
        self.N = Point.N
        Point.N += 1
        self.order = 0
    
    def subhessian(self, R1, R2):
        #self.h = outer(R1, R2)/abs(cross(R1,R2))
        self.h = outer(R1, R2)/(R1@(Rpi2@R2))
    
    def plot(self):
        plt.plot(self.P[0], self.P[1], 'bo')
        #plt.text(self.P[0], self.P[1], str(self.order))   
        
class Graph():
    def __init__(self):
        self.segments = []
        self.circles = []
        Circle.N = 0
        Segment.N = 0
    
    def clean_graph(self):
        for circle in self.circles:
            circle.clean()
        for segment in self.segments:
            segment.clean()
        Point.N  = 0
    
    def compute_gradients(self):
        self.clean_graph()
        for i in range(len(self.circles)):
            for j in range(i+1,len(self.circles)):
                self.circles[i].CC_intersection(self.circles[j])
            for j in range(len(self.segments)):
                self.circles[i].CS_intersection(self.segments[j])
        
        for circle in self.circles:
            circle.filter_points()
            circle.compute_gradient()
        for segment in self.segments:
            segment.filter_points()
    
    def polygon(self, lst):
        for i in range(len(lst)):
            self.segments.append(Segment(lst[i], lst[(i+1)%len(lst)]))
    
    def plot_positions(self):
        for circle in self.circles:
            plt.plot(circle.C[0], circle.C[1], 'r+')
    
    def plot(self):
        for circle in self.circles:
            circle.plot()
            circle.integrate(plot=True)
            for point in circle.points:
                point.plot()
        for segment in self.segments:
            segment.plot()
            segment.integrate(plot=True)
            for point in segment.points:
                point.plot()
    
    def integral(self):
        integral = 0
        for circle in self.circles:
            integral += circle.integrate()
        for segment in self.segments:
            integral += segment.integrate()
        return integral
    
    def graph_gradient(self):
        gradient = zeros(2*len(self.circles))
        for i in range(len(self.circles)):
            gradient[2*i:2*i+2] = self.circles[i].gradient
        return gradient
    
    def graph_hessian(self):
        H = zeros((2*len(self.circles), 2*len(self.circles)))
        for circle in self.circles:
            for point in circle.points:
                SH = zeros((2*len(self.circles), 2*len(self.circles)))
                if(type(point.Cin) == type(point.Cout)):
                    i = point.Cin.N; j = point.Cout.N
                    SH[2*i:2*i+2,2*i:2*i+2] = -point.h
                    SH[2*i:2*i+2,2*j:2*j+2] = point.h
                    SH[2*j:2*j+2,2*i:2*i+2] = point.h.T
                    SH[2*j:2*j+2,2*j:2*j+2] = -point.h.T
                elif(point.Cin == circle):
                    i = circle.N
                    SH[2*i:2*i+2,2*i:2*i+2] = -2*point.h
                else:
                    j = circle.N
                    SH[2*j:2*j+2,2*j:2*j+2] = -2*point.h.T
                H += SH
                    
        return H/2

def plot(x, polygon):
    global iii  # Add this line to access the global variable iii
    graph = Graph()
    graph.polygon(array(polygon))
    positions = np.reshape(x, (-1,2))

    for position in positions:
        graph.circles.append(Circle(position, R))
    graph.compute_gradients()
    graph.plot()
    plt.axis('equal')
    plt.show()
            
def function(x, polygon):
    graph = Graph()
    graph.polygon(array(polygon))
    positions = np.reshape(x, (-1,2))
    for position in positions:
        graph.circles.append(Circle(position, R))
        
    graph.compute_gradients()
    #graph.plot_positions()
    return graph.integral(), graph.graph_gradient(), graph.graph_hessian()


class Algorithm(Node):
    def __init__(self):
        self.polygon = np.array([[1,1],[-1,1],[-1,-1],[1,-1]])*0.9
        self.poses = []
        self.oldposes = self.poses
        self.oldtime = 0.0
        
        self.velocities = []
        
        super().__init__('algorithm')
        self.polygon_subscriber = self.create_subscription(
            PolygonStamped,
            'polygon_ver',
            self.polygon_callback,
            10)
        self.pose_subscriber = self.create_subscription(
            TFMessage,
            'tf',
            self.pose_callback,
            10)
        self.target_publisher = self.create_publisher(FullStateArray, 'target', 10)
        self.timer = self.create_timer(0.01, self.send_cmd)
    
    def polygon_callback(self, msg):
        self.polygon = [[i.x,i.y] for i in msg.polygon.points]
        
    def pose_callback(self, msg):
        self.poses = np.array([[i.transform.translation.x, i.transform.translation.y] for i in msg.transforms])
        t = msg.transforms[0].header.stamp.sec + msg.transforms[0].header.stamp.nanosec*1e-9
        if(len(self.oldposes) != 0):
            self.velocities = (self.poses - self.oldposes)/(t-self.oldtime)
        self.oldtime = t
        self.oldposes = self.poses
    
    def send_cmd(self):
        if(len(self.polygon) == 0 or len(self.velocities) == 0 or len(self.poses) == 0):
            print('cannot start algorithm if no polygon, poses or velocities are present')
            return
        
        F,G,H = function(self.poses, list(reversed(self.polygon)))
        if(True or self.get_clock().now().nanoseconds*1e-9 < 10.0):
            Func.append(F)
            Time.append(self.get_clock().now().nanoseconds*1e-9)
            x.append(self.poses[0][0])
            y.append(self.poses[0][1])
            dx.append(self.velocities[0][0])
            dy.append(self.velocities[0][1])
            Cx.append(G[0])
            Cy.append(G[1])
            
        G *= 0.2
        H *= 0.2
        G = G.reshape((-1,2))
        # for i,g in enumerate(G):
        #     if(linalg.norm(g) !=0):
        #         G[i] = g/linalg.norm(g)*min(1,linalg.norm(g))
        
        points = np.array(self.poses).reshape((-1,2))
        speeds = G
        print("iteration")
        print("\t", np.round(self.velocities.reshape(-1),3))
        print("\t", np.round(G.reshape(-1),3))
        acc = -H@(self.velocities.reshape(-1))
        acc = acc.reshape((-1,2))
        
        target_msg = FullStateArray()
        for i in range(len(points)):
            point = points[i]
            speed = speeds[i]
            acceleration = acc[i]
            
            target_msg.fullstates.append(FullState())
            
            target_msg.fullstates[-1].pose.position.x = float(point[0])
            target_msg.fullstates[-1].pose.position.y = float(point[1])
            target_msg.fullstates[-1].pose.position.z = 1.0
            
            target_msg.fullstates[-1].twist.linear.x = float(speed[0])
            target_msg.fullstates[-1].twist.linear.y = float(speed[1])
            target_msg.fullstates[-1].twist.linear.z = 0.0
            
            target_msg.fullstates[-1].acc.x = acceleration[0] if amorti else 0.0
            target_msg.fullstates[-1].acc.y = acceleration[1] if amorti else 0.0
            target_msg.fullstates[-1].acc.z = 0.0
            
        self.target_publisher.publish(target_msg)
            
        # target_msg = PoseArray()
        # target_msg.header.frame_id = 'world'
        # for i in range(len(points)):
        #     point = points[i]
        #     speed = speeds[i]
        #     target_msg.poses.append(Pose())
        #     target_msg.poses[-1].position.x = float(point[0])
        #     target_msg.poses[-1].position.y = float(point[1])
        #     target_msg.poses[-1].position.z = 1.0
        #     target_msg.poses[-1].orientation.x = speed[0]
        #     target_msg.poses[-1].orientation.y = speed[1]
        #     target_msg.poses[-1].orientation.z = 0.0
        #     target_msg.poses[-1].orientation.w = 1.0
        # self.target_publisher.publish(target_msg)

def main(args=None):
    try:
        rclpy.init(args=args)

        algorithm = Algorithm()

        rclpy.spin(algorithm)

        algorithm.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        plt.plot(Time, Func)
        plt.savefig("/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/scripts/function.pdf")
        np.savetxt("/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/scripts/"+('' if amorti else 'non_')+"amorti.txt",array([Time, Func,x,y,dx,dy, Cx, Cy]).T)
    
if __name__ == '__main__':
    main()
