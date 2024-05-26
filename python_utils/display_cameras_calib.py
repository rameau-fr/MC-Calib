import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2
import pdb

from mpl_toolkits.mplot3d import proj3d

def axisEqual3D(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)

# CameraSize: half of the camera body length
# CameraEdge: line width of the camera body
# CameraColor: 3-vector RGB color of the camera
def Func_cameraDisplay(rot_cam, trans_cam, cam_size, cam_edge, cam_color, ax):
    r = cam_size
    t = trans_cam
    R = rot_cam

    # Corners of the camera in the camera coordinate system 
    X1 = np.asarray([[r,-r,-2*r]]).T
    X2 = np.asarray([[r,r,-2*r]]).T
    X3 = np.asarray([[-r,-r,-2*r]]).T
    X4 = np.asarray([[-r,r,-2*r]]).T
    
    X5 = np.asarray([[r,-r,2*r]]).T
    X6 = np.asarray([[r,r,2*r]]).T
    X7 = np.asarray([[-r,-r,2*r]]).T
    X8 = np.asarray([[-r,r,2*r]]).T
    
    X9 = np.asarray([[1.5*r,-1.5*r,3*r]]).T
    X10 = np.asarray([[1.5*r,1.5*r,3*r]]).T
    X11 = np.asarray([[-1.5*r,-1.5*r,3*r]]).T
    X12 = np.asarray([[-1.5*r,1.5*r,3*r]]).T

    X13 = np.asarray([[0, 0, 0]]).T
    X14 = np.asarray([[0, 0, 2*r]]).T
    X15 = np.asarray([[0, 2*r, 0]]).T
    X16 = np.asarray([[2*r, 0, 0]]).T

    # Corners of the camera in the world coordinate system
    X1 = np.matmul(R,X1) + t
    X2 = np.matmul(R,X2) + t
    X3 = np.matmul(R,X3) + t
    X4 = np.matmul(R,X4) + t
    X5 = np.matmul(R,X5) + t
    X6 = np.matmul(R,X6) + t
    X7 = np.matmul(R,X7) + t
    X8 = np.matmul(R,X8) + t
    X9 = np.matmul(R,X9) + t
    X10 = np.matmul(R,X10) + t
    X11 = np.matmul(R,X11) + t
    X12 = np.matmul(R,X12) + t
    X13 = np.matmul(R,X13) + t
    X14 = np.matmul(R,X14) + t
    X15 = np.matmul(R,X15) + t
    X16 = np.matmul(R,X16) + t

    #plot
    
    ax.plot([X1[0][0], X2[0][0]], [X1[1][0], X2[1][0]],zs=[X1[2][0], X2[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X1[0][0], X3[0][0]], [X1[1][0], X3[1][0]],zs=[X1[2][0], X3[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X4[0][0], X3[0][0]], [X4[1][0], X3[1][0]],zs=[X4[2][0], X4[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X2[0][0], X4[0][0]], [X2[1][0], X4[1][0]],zs=[X2[2][0], X4[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)

    ax.plot([X5[0][0], X6[0][0]], [X5[1][0], X6[1][0]],zs=[X5[2][0], X6[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X5[0][0], X7[0][0]], [X5[1][0], X7[1][0]],zs=[X5[2][0], X7[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X8[0][0], X7[0][0]], [X8[1][0], X7[1][0]],zs=[X8[2][0], X7[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X6[0][0], X8[0][0]], [X6[1][0], X8[1][0]],zs=[X6[2][0], X8[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)

    ax.plot([X1[0][0], X5[0][0]], [X1[1][0], X5[1][0]],zs=[X1[2][0], X5[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X2[0][0], X6[0][0]], [X2[1][0], X6[1][0]],zs=[X2[2][0], X6[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X3[0][0], X7[0][0]], [X3[1][0], X7[1][0]],zs=[X3[2][0], X7[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X4[0][0], X8[0][0]], [X4[1][0], X8[1][0]],zs=[X4[2][0], X8[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)

    ax.plot([X9[0][0], X10[0][0]], [X9[1][0], X10[1][0]],zs=[X9[2][0], X10[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X9[0][0], X11[0][0]], [X9[1][0], X11[1][0]],zs=[X9[2][0], X11[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X12[0][0], X11[0][0]], [X12[1][0], X11[1][0]],zs=[X12[2][0], X11[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X10[0][0], X12[0][0]], [X10[1][0], X12[1][0]],zs=[X10[2][0], X12[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)

    ax.plot([X5[0][0], X9[0][0]], [X5[1][0], X9[1][0]],zs=[X5[2][0], X9[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X6[0][0], X10[0][0]], [X6[1][0], X10[1][0]],zs=[X6[2][0], X10[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X7[0][0], X11[0][0]], [X7[1][0], X11[1][0]],zs=[X7[2][0], X11[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)
    ax.plot([X8[0][0], X12[0][0]], [X8[1][0], X8[1][0]],zs=[X8[2][0], X12[2][0]], color = (cam_color[0],cam_color[1],cam_color[2]), linewidth=cam_edge)

    ax.plot([X13[0][0], X14[0][0]], [X13[1][0], X14[1][0]],zs=[X13[2][0], X14[2][0]], color = (1.0,0.0,0.0), linewidth=cam_edge)
    ax.plot([X13[0][0], X15[0][0]], [X13[1][0], X15[1][0]],zs=[X13[2][0], X15[2][0]], color = (0.0,1.0,0.0), linewidth=cam_edge)
    ax.plot([X13[0][0], X16[0][0]], [X13[1][0], X16[1][0]],zs=[X13[2][0], X16[2][0]], color = (0.0,0.0,1.0), linewidth=cam_edge)

#!!!! This display code assumes a single camera group remain at the end the calibration process
# 1. path to calibration results
#path_root = "/home/francois/Documents/CppProject/Stereo_Calibration/Images_Synth_4cam_8boards_4groups_NonOverlap/"
#path_root = "/disk/francois/Projets/Calibration_toolbox/Data/Real_Images/2021-07-12-15-09-53/"
#path_root = "/disk/francois/Projets/Calibration_toolbox/Data/Real_Images/stereo_cube/"
path_root = "/disk/francois/Projets/Calibration_toolbox/Data/Real_Images/2021-07-12-15-09-53/"
path_calib_results = path_root + "calibrated_cameras_data.yml"
cam_size = 3
cam_edge = 2
cam_color = [0.0, 0.0, 0.0]
# 2. Open the file
fs = cv2.FileStorage(path_calib_results, cv2.FILE_STORAGE_READ)
Nb_Camera = fs.getNode("nb_camera").real()
Nb_Camera = int(Nb_Camera)

#2. declare figure
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_proj_type('ortho')
ax.set_title('Calibration Result')

# 3. Read the data for all the cameras
for i in range(0,Nb_Camera):
    cam_id = "camera_" + str(i)
    cam_mat = fs.getNode(cam_id).getNode("camera_matrix").mat()
    cam_pose = fs.getNode(cam_id).getNode("camera_pose_matrix").mat()
    #cam_pose = np.linalg.inv(cam_pose)
    t = np.asarray([cam_pose[0:3,3]]).T
    R = cam_pose[0:3,0:3]
    Func_cameraDisplay(R, t, cam_size, cam_edge, cam_color,ax)
    print(cam_pose)

#ax.axis('equal')
#ax.set_aspect('equal', 'box')
#fig.tight_layout()
#ax.view_init(20, 90)
ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
axisEqual3D(ax)
#ax.set_xlim3d(-5, 3)
#ax.set_ylim3d(1, -12)
#ax.set_zlim3d(-1.5, 1.5)
ax.set_xlabel('x (cm)',fontsize=20)
ax.set_ylabel('y (cm)',fontsize=20)
ax.set_zlabel('z (cm)',fontsize=20)
plt.show()

# 2. Display the cameras
