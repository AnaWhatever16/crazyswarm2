#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, PoseArray, Pose
from tf2_msgs.msg import TFMessage
import numpy as np
from crazyflie_interfaces.msg import FullStateArray, FullState, LogDataGeneric
from copy import deepcopy 

import time
import yaml

from numpy import *
import matplotlib.pyplot as plt
import os
amorti = True

Time = []
Func = []
x = []
y = []
dx = []
dy = []
Cx = []
Cy = []
goalposex = []
goalposey = []
ax = []
ay = []

data = []

R = 1.0
def is_pos_def(x):
    return np.all(np.linalg.eigvals(x) >= 0)

def diag_blocks(n):
    return np.eye(2*n) + np.diag([i%2 for i in range(1,2*n)], k =1) + np.diag([i%2 for i in range(1,2*n)], k =-1)

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

            
    def remove_inner_points(self):
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

    
    def integrate(self, segments, plot=False):
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
            integral -= self.r**2*(theta2-theta1)/2 + np.cross(self.C, edge[1] - edge[0])/2
        if(self.isolated):
            vector = np.array([self.C, np.array([1.0,0])])
            intersections = []
            for segment in segments:
                A = segment.P1
                B = segment.P2
                AB = B - A
                if(np.cross(vector[1], AB)):
                    t = np.cross(A - vector[0], vector[1]) / np.cross(vector[1], AB)
                    if 0 <= t <= 1:
                        intersection = A + t * AB
                        intersections.append(intersection)    
            intersections = [x for x in list(map(lambda x: (x - self.C)@(np.array([1.0,0]))/self.r, intersections))]
            intersections = [x for x in list(filter(lambda x: x < -1, intersections))]
            if(len(intersections) %2 == 1):
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
            if(self.points[i].Cin == None):
                for j in range(0,i):
                    self.points[j].order += 1
                    
    def remove_inner_points(self):
        self.points = [x for x in filter(lambda x: x.order == 0, self.points)]

    def integrate(self, plot = False):
        integral = 0
        for i in range(1,len(self.points),2):
            AA = self.points[i-1].P
            BB = self.points[i].P
            if(plot):
                plt.plot([self.points[i-1].P[0], self.points[i].P[0]],[self.points[i-1].P[1], self.points[i].P[1]], 'b')
            integral += (AA[1]*BB[0] - AA[0]*BB[1])/2
            
        return integral
    
    def plot(self):
        plt.plot([self.P1[0], self.P2[0]],[self.P1[1], self.P2[1]], 'r')
        plt.text((self.P1[0] + self.P2[0])/2,(self.P1[1] + self.P2[1])/2,str(self.N))

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
        plt.text(self.P[0], self.P[1], str(self.order))   
        
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
        
        for segment in self.segments:
            segment.filter_points()
        for circle in self.circles:
            circle.filter_points()
            
        for segment in self.segments:
            segment.remove_inner_points()
        for circle in self.circles:
            circle.remove_inner_points()
            
        for circle in self.circles:
            circle.compute_gradient()
    
    def polygon(self, lst):
        for i in range(len(lst)):
            self.segments.append(Segment(lst[i], lst[(i+1)%len(lst)]))
    
    def plot_positions(self):
        for circle in self.circles:
            plt.plot(circle.C[0], circle.C[1], 'r+')
    
    def plot(self):
        for segment in self.segments:
            segment.plot()
            segment.integrate(plot=True)
            for point in segment.points:
                point.plot()
        for circle in self.circles:
            circle.plot()
            circle.integrate(self.segments, plot=True)
            for point in circle.points:
                point.plot()
        
    
    def integral(self):
        integral = 0
        for circle in self.circles:
            integral += circle.integrate(self.segments)
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


def plot(x):
    graph = Graph()
    graph.polygon(array([[-1,-1],[-1,1],[1,1],[1,-1]]))
    positions = np.reshape(x, (-1,2))

    for position in positions:
        graph.circles.append(Circle(position, R))
    graph.compute_gradients()
    graph.plot()

    plt.axis('equal')
    plt.show()
            
def function(x, polygon):
    graph = Graph()
    graph.polygon(polygon)
    positions = np.reshape(x, (-1,2))

    for position in positions:
        graph.circles.append(Circle(position, R))
        
    graph.compute_gradients()
    #graph.plot_positions()
    return graph.integral(), graph.graph_gradient(), graph.graph_hessian()
    

class Algorithm(Node):
    def __init__(self):
        self.polygon = None
        
        with open('/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml', 'r') as file:
            config = yaml.safe_load(file)
        self.cf_names = ([cf_name for cf_name in list(filter(lambda x: config['robots'][x]['enabled'],config['robots'].keys()))])
        self.index_of_cf = {}
        for i, cf_name in enumerate(self.cf_names):
            self.index_of_cf[cf_name] = i
        
        self.poses = [None] * (2 * len(self.cf_names))
        self.velocities = [None] * (2 * len(self.cf_names))
        self.accelerations = [None] * (2 * len(self.cf_names))
        
        super().__init__('algorithm')
        self.polygon_subscriber = self.create_subscription(
            PolygonStamped,
            'polygon_ver',
            self.polygon_callback,
            10)

        self.subList = {}
        for cf_name in self.cf_names:
            self.subList[cf_name] = self.create_subscription(
                LogDataGeneric,
                '/' + cf_name + '/posVelAcc',
                lambda msg, n=cf_name: self.pose_callback(msg, n),
                10)
        
        self.target_publisher = self.create_publisher(FullStateArray, 'target', 10)
        self.timer = self.create_timer(0.01, self.send_cmd)
    
    def polygon_callback(self, msg):
        self.polygon = [[i.x,i.y] for i in msg.polygon.points]
    
    def pose_callback(self, msg, NAME):
        # write positions
        self.poses[self.index_of_cf[NAME]*2] = msg.values[0]/1000
        self.poses[self.index_of_cf[NAME]*2+1] = msg.values[1]/1000
        
        # write velocities
        self.velocities[self.index_of_cf[NAME]*2] = msg.values[2]/1000
        self.velocities[self.index_of_cf[NAME]*2+1] = msg.values[3]/1000
        
        # write accelerations
        self.accelerations[self.index_of_cf[NAME]*2] = msg.values[4]/1000
        self.accelerations[self.index_of_cf[NAME]*2+1] = msg.values[5]/1000
    
    def send_cmd(self):
        if(None in self.poses or self.polygon == None):
            return
    
        
        F,G,H = function(self.poses, list(reversed(self.polygon)))
        factor = 1/(2*R)
        G *= factor
        H *= factor*diag_blocks(len(self.cf_names))
        
        # logging
        if(True):
            Func.append(F)
            Time.append(self.get_clock().now().nanoseconds*1e-9)
            x.append(self.poses[0])
            y.append(self.poses[1])
            dx.append(self.velocities[0])
            dy.append(self.velocities[1])
            Cx.append(G[0])
            Cy.append(G[1])
            goalposex.append(self.polygon[0][0]-0.9)
            goalposey.append(self.polygon[0][1]-0.9)
            ax.append(self.accelerations[0])
            ay.append(self.accelerations[1])
            
            data.append([self.get_clock().now().nanoseconds*1e-9, F] + self.poses)

        Vref = G
        Aref = -H@(self.velocities)
        
        target_msg = FullStateArray()
        for i in range(len(self.cf_names)):
            vref = Vref[2*i:2*i+2]
            vmes = self.velocities[2*i:2*i+2]
            aref = Aref[2*i:2*i+2]
            
            target_msg.fullstates.append(FullState())
            target_msg.fullstates[-1].header.frame_id = self.cf_names[i]
            
            target_msg.fullstates[-1].pose.position.x = 10.0
            target_msg.fullstates[-1].pose.position.y = 10.0
            target_msg.fullstates[-1].pose.position.z = 0.5
                        
            target_msg.fullstates[-1].twist.linear.x = 0.0
            target_msg.fullstates[-1].twist.linear.y = 0.0
            target_msg.fullstates[-1].twist.linear.z = 0.0
            
            Gain = 0.72
            kp = 1.78
                        
            target_msg.fullstates[-1].acc.x = np.clip(float(aref[0]/Gain +  kp*(vref[0] - vmes[0])), -1.0,1.0)
            target_msg.fullstates[-1].acc.y = np.clip(float(aref[1]/Gain +  kp*(vref[1] - vmes[1])), -1.0,1.0)
            target_msg.fullstates[-1].acc.z = 0.0
        
        self.target_publisher.publish(target_msg)

        
            
def main(args=None):
    try:
        rclpy.init(args=args)

        algorithm = Algorithm()

        rclpy.spin(algorithm)

        algorithm.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        TIME = time.localtime()
        TIME = str(TIME.tm_year) + "_" + str(TIME.tm_mon) + "_" + str(TIME.tm_mday) + "_" + str(TIME.tm_hour) + "h" + str(TIME.tm_min) + "m" + str(TIME.tm_sec) + "s"
        path = os.path.join("/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results", TIME) 
        os.mkdir(path)
        #np.savetxt(os.path.join(path,('' if amorti else 'non_') + 'amorti.txt'),array([Time,Func,x,y,goalposex,goalposey,dx,dy, Cx, Cy, ax, ay]).T)
        np.savetxt(os.path.join(path,('' if amorti else 'non_') + 'amorti.txt'),array(data))
      
if __name__ == '__main__':
    main()
