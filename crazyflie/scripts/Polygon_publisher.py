#!/usr/bin/env python3

import pygame
import sys
from geometry_msgs.msg import Point32,Polygon,PolygonStamped
import numpy as np

import yaml

def distance(point1, point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

import time

class Polygon_publisher(Node):
    def __init__(self):
        super().__init__('Polygon')
        self.publisher_ = self.create_publisher(PolygonStamped, 'polygon_ver', 10)
        self.pose_subscriber = self.create_subscription(
            TFMessage,
            'tf',
            self.positions_update,
            1)
        self.i = 0
        with open('/coverage_crazyflie_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml', 'r') as file:
            config = yaml.safe_load(file)
        self.cf_names = ([cf_name for cf_name in list(filter(lambda x: config['robots'][x]['enabled'],config['robots'].keys()))])
        self.index_of_cf = {}
        for i, cf_name in enumerate(self.cf_names):
            self.index_of_cf[cf_name] = i
        
        self.poses = [[0,0]] * (len(self.cf_names))
        
        pygame.init()
        self.M,self.N = 1000,1000
        self.ecran = pygame.display.set_mode((self.M, self.N))
        self.font = pygame.font.SysFont(None, 20)  # Create a font object for displaying text
        self.points = np.array([[1, -1], [-1, -1], [-1, 1], [1, 1]])*100
        for i in range(len(self.points)):
            self.points[i] = (self.points[i][0] + self.M//2, self.points[i][1] + self.N//2)
        self.couleur_polygone = (0, 255, 0, 128)
        self.couleur_selection = (255, 0, 0)
        self.rayon_selection = 5
        self.point_selectionne = None
        self.polygone_selectionne = False
        self.i = 0
        

    
    def polygon_window(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            
            if event.type == pygame.MOUSEBUTTONDOWN:
                pos_souris = pygame.mouse.get_pos()
                self.polygone_selectionne = True
                for i, point in enumerate(self.points):
                    if distance(pos_souris, point) <= self.rayon_selection:
                        self.point_selectionne = i
                        self.polygone_selectionne = False
                        break

        
            if event.type == pygame.MOUSEBUTTONUP:
                self.point_selectionne = None
                self.polygone_selectionne = False

        
            if event.type == pygame.MOUSEMOTION:
                if self.polygone_selectionne:
                
                    for i in range(len(self.points)):
                        self.points[i] = (self.points[i][0] + event.rel[0], self.points[i][1] + event.rel[1])
                elif self.point_selectionne is not None:
                
                    self.points[self.point_selectionne] = pygame.mouse.get_pos()

            self.ecran.fill((255, 255, 255))

                

            # Dessiner les axes x et y
            pygame.draw.line(self.ecran, (255, 0, 0), (0, self.N//2), (self.M, self.N//2), 2)  # Axe x (rouge)
            pygame.draw.line(self.ecran, (0, 0, 255), (self.M/2, 0), (self.M//2, self.N), 2)  # Axe y (bleu)
            
            for pose in self.poses:
                pygame.draw.circle(self.ecran, (0, 0, 255), (pose[0]*100 + self.M//2,-pose[1]*100+self.N//2), 100)
            
            pygame.draw.polygon(self.ecran, self.couleur_polygone, self.points, width = 10)

            for i, point in enumerate(self.points):
                if i == self.point_selectionne:
                    couleur = self.couleur_selection
                else:
                    couleur = (0, 0, 0)
                pygame.draw.circle(self.ecran, couleur, point, self.rayon_selection)
                coordinates = (np.array(self.points)-np.array([[self.M//2, self.N//2]] * (len(self.points))))/100 * np.array([[1, -1]] * (len(self.points)))
                self.send_message(coordinates)
                # Create text surface with coordinates and render it next to the point
                coord_text = self.font.render(f"({(point[0]-self.M//2)/100}, {(point[1]-self.N//2)/100})", True, (0, 0, 0))  # Black text
                text_rect = coord_text.get_rect(center=point)  # Center the text next to the point
                self.ecran.blit(coord_text, text_rect)
            for x in range(0, self.M, 100):  # Lignes verticales
                pygame.draw.line(self.ecran, (200, 200, 200), (x, 0), (x, self.N))
            for y in range(0, self.N, 100):  # Lignes horizontales
                pygame.draw.line(self.ecran, (200, 200, 200), (0, y), (self.M, y))
    
        
            pygame.display.update()
        
    
    def positions_update(self, msg):
        for transform in msg.transforms:
            self.poses[self.index_of_cf[transform.child_frame_id]] = [transform.transform.translation.x,transform.transform.translation.y]
        self.polygon_window()
        self.i += 1
    
    def send_message(self, data):
        polygon = Polygon()
        for point in data:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            polygon.points.append(p)
        msg = PolygonStamped()
        msg.polygon = polygon
        msg.header.frame_id = 'world'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    polygon_publisher = Polygon_publisher()

    rclpy.spin(polygon_publisher)

    polygon_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


































# import pygame
# import sys
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import Point32,Polygon,PolygonStamped
# import numpy as np
# from rclpy.executors import ExternalShutdownException


# def distance(point1, point2):
#     return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5

# class Polygon_publisher(Node):
#     def __init__(self):
#         super().__init__('Polygon')
#         self.publisher_ = self.create_publisher(PolygonStamped, 'polygon_ver', 10)
        
#         self.i = 0
#         pygame.init()
#         M,N = 1000,1000
#         ecran = pygame.display.set_mode((M, N))
#         font = pygame.font.SysFont(None, 20)  # Create a font object for displaying text
#         points = [(100, 100), (100, -100), (-100, -100), (-100, 100)]
#         for i in range(len(points)):
#             points[i] = (points[i][0] + M//2, points[i][1] + N//2)
#         couleur_polygone = (0, 255, 0, 128)
#         couleur_selection = (255, 0, 0)
#         rayon_selection = 10
#         point_selectionne = None
#         polygone_selectionne = False
#         while True:
#             for event in pygame.event.get():
#                 if event.type == pygame.QUIT:
#                     pygame.quit()
#                     sys.exit()

                
#                 if event.type == pygame.MOUSEBUTTONDOWN:
#                     pos_souris = pygame.mouse.get_pos()
#                     polygone_selectionne = True
#                     for i, point in enumerate(points):
#                         if distance(pos_souris, point) <= rayon_selection:
#                             point_selectionne = i
#                             polygone_selectionne = False
#                             break

            
#                 if event.type == pygame.MOUSEBUTTONUP:
#                     point_selectionne = None
#                     polygone_selectionne = False

            
#                 if event.type == pygame.MOUSEMOTION:
#                     if polygone_selectionne:
                    
#                         for i in range(len(points)):
#                             points[i] = (points[i][0] + event.rel[0], points[i][1] + event.rel[1])
#                     elif point_selectionne is not None:
                    
#                         points[point_selectionne] = pygame.mouse.get_pos()

#             ecran.fill((255, 255, 255))

#             for x in range(0, M, 100):  # Lignes verticales
#                 pygame.draw.line(ecran, (200, 200, 200), (x, 0), (x, N))
#             for y in range(0, N, 100):  # Lignes horizontales
#                 pygame.draw.line(ecran, (200, 200, 200), (0, y), (M, y))

#             # Dessiner les axes x et y
#             pygame.draw.line(ecran, (255, 0, 0), (0, N//2), (M, N//2), 2)  # Axe x (rouge)
#             pygame.draw.line(ecran, (0, 0, 255), (M/2, 0), (M//2, N), 2)  # Axe y (bleu)

#             pygame.draw.polygon(ecran, couleur_polygone, points, width = 10)

#             for i, point in enumerate(points):
#                 if i == point_selectionne:
#                     couleur = couleur_selection
#                 else:
#                     couleur = (0, 0, 0)
#                 pygame.draw.circle(ecran, couleur, point, rayon_selection)
#                 coordinates = (np.array(points)-np.array([[M//2, N//2]] * (len(points))))/100 * np.array([[1, -1]] * (len(points)))
#                 self.send_message(coordinates)
#                 # Create text surface with coordinates and render it next to the point
#                 coord_text = font.render(f"({(point[0]-M//2)/100}, {(point[1]-N//2)/100})", True, (0, 0, 0))  # Black text
#                 text_rect = coord_text.get_rect(center=point)  # Center the text next to the point
#                 ecran.blit(coord_text, text_rect)
                
#             pygame.display.update()
        
        
#     def send_message(self, data):
#         polygon = Polygon()
#         for point in data:
#             p = Point32()
#             p.x = float(point[0])
#             p.y = float(point[1])
#             polygon.points.append(p)
#         msg = PolygonStamped()
#         msg.polygon = polygon
#         msg.header.frame_id = 'world'
#         self.publisher_.publish(msg)


# def main(args=None):
#     rclpy.init()

#     minimal_publisher = Polygon_publisher()
#     try:
#         rclpy.spin(minimal_publisher)
#     except ExternalShutdownException:
#         pass

    
    
#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
    
   
        