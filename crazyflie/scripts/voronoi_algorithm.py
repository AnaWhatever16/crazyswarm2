import numpy as np
import quaternion
from tf_transformations import quaternion_from_euler, quaternion_multiply

# Quaternion original
w = 1
x = 0
y = 0
z = 0

# Quaternion de rotation de 90 degrés autour de l'axe x
q = np.array([w, x, y, z])  # Remplacez w, x, y, z par les valeurs de votre quaternion original

# Quaternion de rotation de 90 degrés autour de l'axe x
theta = np.pi / 2  # 90 degrés en radians

q1 = quaternion_multiply(quaternion_from_euler(np.pi/4, 0, 0),quaternion_from_euler(np.pi/4, 0, 0))

print(list(q1))





# from foronoi import Voronoi, Polygon, Visualizer, VoronoiObserver
# import numpy as np
# import matplotlib.pyplot as plt
# from numpy import array

# # Define some points (a.k.a sites or cell points)
# points = np.array([
#     (2.5, 2.5),
#     (4, 7.5),
#     (7.5, 2.5),
#     (6, 7.5),
#     (4, 4),
#     (3, 3),
#     (6, 3),
# ]) 

# # Define a bounding box / polygon
# polygon = Polygon([
#     (2.5, 10),
#     (5, 10),
#     (10, 5),
#     (10, 2.5),
#     (5, 0),
#     (2.5, 0),
#     (0, 2.5),
#     (0, 5),
# ])

# def diagram(points):
#     v = Voronoi(polygon)
#     v.create_diagram(points)

    
#     for site in v.sites:
#         plt.plot(site.x, site.y, 'bo')
#         edge = site.first_edge
#         vertices = []
#         while(edge!= site.first_edge.next or len(vertices) == 1):
#             vertices.append([edge.origin.x, edge.origin.y])
#             edge = edge.next
#         plt.plot(np.array(vertices).T[0], np.array(vertices).T[1])
#     plt.axis('equal')
#     plt.show()
    
    

# diagram(points)
# diagram(points)


# # def center_of_mass(vertices):
# #     vx = vertices.T[0]
# #     vy = vertices.T[1]
# #     A = 0.5*sum(vx[1:]*vy[:-1] - vx[:-1]*vy[1:])
# #     xG = 1/(6*A)*sum( (vx[:-1] + vx[1:])*(vx[1:]*vy[:-1] - vx[:-1]*vy[1:]))
# #     yG = 1/(6*A)*sum( (vy[:-1] + vy[1:])*(vx[1:]*vy[:-1] - vx[:-1]*vy[1:]))
# #     return np.array([xG,yG])


# # def compute_gradient(points, plot = False):
# #     v = Voronoi(polygon)
# #     v.create_diagram(points=points)
    
# #     cells = []
# #     gradient = np.zeros(points.shape)
# #     for i, site in enumerate(v.sites):
# #         vertices = [[site.first_edge.origin.x, site.first_edge.origin.y],[site.first_edge.next.origin.x, site.first_edge.next.origin.y]]
# #         edge = site.first_edge.next.next
# #         while(edge != site.first_edge.next):
# #             vertices.append([edge.origin.x, edge.origin.y])
# #             edge = edge.next
# #         cells.append(vertices)
# #         center_of_mass(np.array(vertices))
        
# #         position = np.array([site.x, site.y])
# #         center = center_of_mass(np.array(vertices))
# #         gradient[i] = center - position
# #         if(plot):
# #             plt.plot(np.array(vertices).T[0], np.array(vertices).T[1])
# #             plt.plot(site.x, site.y, 'bo')
# #             plt.plot(center[0], center[1], 'yo')
# #     return gradient
