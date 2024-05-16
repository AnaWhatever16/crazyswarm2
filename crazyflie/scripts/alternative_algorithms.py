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
from foronoi import Voronoi, Polygon

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

acc_x = []
acc_y = []

data = []

R = 1.0

def center_of_mass(vertices):
    vx = vertices.T[0]
    vy = vertices.T[1]
    A = 0.5*sum(vx[1:]*vy[:-1] - vx[:-1]*vy[1:])
    xG = 1/(6*A)*sum( (vx[:-1] + vx[1:])*(vx[1:]*vy[:-1] - vx[:-1]*vy[1:]))
    yG = 1/(6*A)*sum( (vy[:-1] + vy[1:])*(vx[1:]*vy[:-1] - vx[:-1]*vy[1:]))
    return np.array([xG,yG])

def centroids(points, polygon, plot = False):
    v = Voronoi(Polygon(polygon))
    v.create_diagram(points)
    gradient = np.zeros((len(points), 2))
    for i,site in enumerate(v.sites):
        edge = site.first_edge
        vertices = []
        while(edge!= site.first_edge.next or len(vertices) == 1):
            vertices.append([edge.origin.x, edge.origin.y])
            edge = edge.next
        G = center_of_mass(np.array(vertices))
        P = np.array([site.x, site.y])
        gradient[i] = G - P
        plt.plot(site.x, site.y, 'rx')
        if(plot):
            plt.plot(site.x, site.y, 'bo')
            plt.text(site.x,site.y, str(site.name))
            plt.plot(np.array(vertices).T[0], np.array(vertices).T[1])
            plt.plot(G[0], G[1],'bo')
    if(plot):
        plt.axis('equal')
        plt.show()
    return gradient

def dist(p1, p2):
    return np.linalg.norm(p1 - p2)
def circle_from_two_points(p1, p2):
    center = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    radius = dist(p1, p2) / 2
    return center, radius
def circle_from_three_points(p1, p2, p3):
    A, B, C = p1, p2, p3
    D = 2 * (A[0]*(B[1]-C[1]) + B[0]*(C[1]-A[1]) + C[0]*(A[1]-B[1]))
    Ux = ((A[0]**2 + A[1]**2)*(B[1] - C[1]) + (B[0]**2 + B[1]**2)*(C[1] - A[1]) + (C[0]**2 + C[1]**2)*(A[1] - B[1])) / D
    Uy = ((A[0]**2 + A[1]**2)*(C[0] - B[0]) + (B[0]**2 + B[1]**2)*(A[0] - C[0]) + (C[0]**2 + C[1]**2)*(B[0] - A[0])) / D
    center = (Ux, Uy)
    radius = dist(center, A)
    return center, radius
def is_in_circle(p, center, radius):
    return dist(p, center) <= radius + 1e-9
def welzl(P, R=[]):
    if not P or len(R) == 3:
        if len(R) == 0: return ((0, 0), 0)
        if len(R) == 1: return (R[0], 0)
        if len(R) == 2: return circle_from_two_points(R[0], R[1])
        return circle_from_three_points(R[0], R[1], R[2])
    
    p = P.pop()
    center, radius = welzl(P, R)
    
    if is_in_circle(p, center, radius):
        P.append(p)
        return center, radius
    
    R.append(p)
    center, radius = welzl(P, R)
    P.append(p)
    R.pop()
    return center, radius
def smallest_enclosing_circle(points):
    P = list(points)
    return welzl(P)[0]
def minimax(points, polygon, plot = False):
    v = Voronoi(Polygon(polygon))
    v.create_diagram(list(points))
    gradient = np.zeros((len(points), 2))
    for i,site in enumerate(v.sites):
        edge = site.first_edge
        vertices = []
        while(edge!= site.first_edge.next or len(vertices) == 1):
            vertices.append([edge.origin.x, edge.origin.y])
            edge = edge.next
        G = smallest_enclosing_circle(np.array(vertices))
        P = np.array([site.x, site.y])
        gradient[i] = G - P
        plt.plot(site.x, site.y, 'rx')
        if(plot):
            plt.plot(site.x, site.y, 'bo')
            plt.text(site.x,site.y, str(site.name))
            plt.plot(np.array(vertices).T[0], np.array(vertices).T[1])
            plt.plot(G[0], G[1],'bo')
    if(plot):
        plt.axis('equal')
        plt.show()
    return gradient


def alternative_algorithm(x, polygon, name):
    poly = polygon
    if name == 'minimax':
        return minimax(x, poly)
    if name == 'centroid':
        return centroids(x, poly)
    else:
        return None
    

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
    
        F = 0
        G = alternative_algorithm(np.array(self.poses).reshape((-1,2)), self.polygon, 'minimax').reshape(-1)*100
        H = np.zeros((len(G), len(G)))
        
                
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
            
            vref = -np.array([self.poses[0], self.poses[1]])*10
            
            Gain = 0.72
            kp = 1.78
                        
            target_msg.fullstates[-1].acc.x = np.clip(float(aref[0]/Gain +  kp*(vref[0] - vmes[0])), -2.0,2.0)
            target_msg.fullstates[-1].acc.y = np.clip(float(aref[1]/Gain +  kp*(vref[1] - vmes[1])), -2.0,2.0)
            target_msg.fullstates[-1].acc.z = 0.0
            
            acc_x.append(target_msg.fullstates[-1].acc.x)
            acc_y.append(target_msg.fullstates[-1].acc.y)
        
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
        path = os.path.join("/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results_alt", TIME) 
        os.mkdir(path)
        np.savetxt(os.path.join(path,('' if amorti else 'non_') + 'amorti_old.txt'),array([Time,Func,x,y,goalposex,goalposey,dx,dy, Cx, Cy, ax, ay, acc_x, acc_y]).T)
        np.savetxt(os.path.join(path,('' if amorti else 'non_') + 'amorti.txt'),array(data))

      
if __name__ == '__main__':
    main()
