import numpy as np
import matplotlib.pyplot as plt

def draw_isometric_projection(points_3d, ax, ax2, col = plt.cm.tab10(0)):
    connections = [(0,1), (0,3), (0,4), (6,2), (6,5), (6,7), (5,1), (5,4), (3,2), (3,7), (1,2), (4,7)]
    
    # isometric projection matrix
    A = np.array([[np.sqrt(3), 0, -np.sqrt(3)],[1,2,1]]) / np.sqrt(6)
    points_2d = np.zeros((points_3d.shape[0], 2))

    ax.scatter(points_3d[:,0], points_3d[:,2], points_3d[:,1], color = col)
    ax.set_xlabel('x0')
    ax.set_ylabel('x2')
    ax.set_zlabel('x1')
    ax.view_init(azim=360-135)
    
    ax.set_xlim(points_3d.min(),points_3d.max())
    ax.set_ylim(points_3d.min(),points_3d.max())
    ax.set_zlim(points_3d.min(),points_3d.max())
    
    for con in connections:
        ax.plot([points_3d[con[0],0], points_3d[con[1],0]], [points_3d[con[0],2], points_3d[con[1],2]], [points_3d[con[0],1], points_3d[con[1],1]], color = col, lw=0.5)

    for i, point_3d in enumerate(points_3d):
        points_2d[i,:] = A @ point_3d
        ax.text(point_3d[0], point_3d[2], point_3d[1], f'{i}')

    ax2.plot(points_2d[:,0], points_2d[:,1], '.', color = col)
    for i, point_2d in enumerate(points_2d):
        ax2.annotate(f'{i}', (point_2d[0], point_2d[1]))


    for con in connections:
        ax2.plot([points_2d[con[0],0], points_2d[con[1],0]], [points_2d[con[0],1], points_2d[con[1],1]], color = col, lw=0.5)
    ax2.set_aspect('equal')
    ax2.grid(ls=':')
    ax2.set_xlabel('x0')
    ax2.set_ylabel('x1')

#--------------------------------------------------------------------------------------------------------------------------

def generate_cuboid_points(width, height, depth, offset, theta):

    # rotation matrix for rotation around x1 axis
    R1 = np.array([[np.cos(theta), 0, np.sin(theta)],[0,1,0], [-np.sin(theta), 0, np.cos(theta)]])

    cuboid_points = np.array([[-0.5*width, -0.5*height, -0.5*depth], 
                           [-0.5*width, -0.5*height, 0.5*depth], 
                           [-0.5*width, 0.5*height, 0.5*depth],
                           [-0.5*width, 0.5*height, -0.5*depth],
                           [0.5*width, -0.5*height, -0.5*depth], 
                           [0.5*width, -0.5*height, 0.5*depth], 
                           [0.5*width, 0.5*height, 0.5*depth],
                           [0.5*width, 0.5*height, -0.5*depth]])

    # rotate cuboid
    cuboid_points_rotated = np.zeros_like(cuboid_points)
    for i, point_3d in enumerate(cuboid_points):
        cuboid_points_rotated[i,:] = R1 @ point_3d

    # translate cuboid
    cuboid_points_rotated[:,0] += offset[0]
    cuboid_points_rotated[:,1] += offset[1]
    cuboid_points_rotated[:,2] += offset[2]
   
    return cuboid_points_rotated



