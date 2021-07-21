import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2
import pdb

from mpl_toolkits.mplot3d import proj3d
def orthogonal_proj(zfront, zback):
    a = (zfront+zback)/(zfront-zback)
    b = -2*(zfront*zback)/(zfront-zback)
    return np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,a,b],
                        [0,0,0,zback]])
#proj3d.persp_transformation = orthogonal_proj


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

def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

def eul2rot(theta) :

    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R



#!!!! This display code assumes a single camera group remain at the end the calibration process

# 1. Open the parameters obtain by the calibration toolbox
# path to calibration results
#path_root = "/home/francois/Documents/CppProject/Stereo_Calibration/Simulation/Scenario_5/Images/"
path_root = "/disk/francois/Projets/Calibration_toolbox/Data/Blender_Images/Scenario_5/Results/"
#path_root = "/disk/francois/Projets/Calibration_toolbox/Data/Real_Images/2021-06-18-15-00-16/"
path_calib_results = path_root + "calibrated_cameras_data.yml"
cam_size = 0.5
cam_edge = 2
cam_color = [0.0, 0.0, 0.0] 
cam_color_gt = [1.0, 0.0, 1.0]
# Open the file
fs = cv2.FileStorage(path_calib_results, cv2.FILE_STORAGE_READ)
Nb_Camera = fs.getNode("nb_camera").real()
Nb_Camera = int(Nb_Camera)

# declare figure
fig = plt.figure()
ax = fig.gca(projection='3d')
#ax.set_title('Calibration Result')

# Read the data for all the cameras and save the results
listR_Est = []
listT_Est = []
listK_Est = []
for i in range(0,Nb_Camera):
    cam_id = "camera_" + str(i)
    cam_mat = fs.getNode(cam_id).getNode("camera_matrix").mat()
    cam_pose = fs.getNode(cam_id).getNode("camera_pose_matrix").mat()
    #cam_pose = np.linalg.inv(cam_pose)
    #pdb.set_trace()
    t = np.asarray([cam_pose[0:3,3]]).T
    R = cam_pose[0:3,0:3]
    Func_cameraDisplay(R, t, cam_size, cam_edge, cam_color,ax)
    listR_Est.append(R)
    listT_Est.append(t)
    listK_Est.append(cam_mat)
    print(cam_pose)

#2. Open the ground truth
path_root_gt = "/home/francois/Documents/CppProject/Stereo_Calibration/Simulation/Scenario_5/"
path_calib_gt = path_root_gt + "GroundTruth.yml"
fs_gt = cv2.FileStorage(path_calib_gt, cv2.FILE_STORAGE_READ)
Nb_Camera = fs_gt.getNode("nb_camera").real()
Nb_Camera = int(Nb_Camera)

listR_GT = []
listT_GT = []
listK_GT = []

for i in range(0,Nb_Camera):
    transf_id = "P_" + str(i+1)
    cam_id = "K_" + str(i+1)
    cam_mat = fs_gt.getNode(cam_id).mat()
    cam_pose = fs_gt.getNode(transf_id).mat()
    
    trans = np.array([[1, 0, 0, 0], [0,-1,0,0], [0,0,-1,0],[0,0,0,1]])
    #pdb.set_trace()
    #cam_pose = np.linalg.inv(trans)@cam_pose@trans
    cam_pose = trans@cam_pose@trans
    cam_pose = np.linalg.inv(cam_pose)
    t = np.asarray([cam_pose[0:3,3]]).T
    R = cam_pose[0:3,0:3]
    #pdb.set_trace()
    '''
    cam_pose[2,3] = -cam_pose[2,3]
    t = np.asarray([cam_pose[0:3,3]]).T
    R = cam_pose[0:3,0:3]
    ang = rot2eul(R)
    ang[1] = - ang[1]
    R = eul2rot(ang)
    '''
    #Func_cameraDisplay(R, t, cam_size, cam_edge, cam_color_gt,ax)
    listR_GT.append(R)
    listT_GT.append(t)
    listK_GT.append(cam_mat)
    print(cam_pose)
    
#3. Compute error
List_err_R = []
List_err_T = []
List_err_fx = []
List_err_fy = []
List_err_u0 = []
List_err_v0 = []
for i in range(0,Nb_Camera):
    R_GT = listR_GT[i]
    R_est = listR_Est[i]
    T_GT = listT_GT[i]
    T_est = listT_Est[i]
    K_GT = listK_GT[i]
    K_est= listK_Est[i]
    K_err = np.sqrt((K_est-K_GT)**2)
    List_err_fx.append(K_err[0][0]) 
    List_err_fy.append(K_err[1][1]) 
    List_err_u0.append(K_err[0][2]) 
    List_err_v0.append(K_err[1][2]) 
    T_error = np.sqrt(np.sum((T_GT - T_est)**2))
    R_error = np.degrees(np.arccos(0.5*(np.trace(np.matmul(np.linalg.inv(R_est),R_GT))-1)))
    if (np.isnan(R_error)):
        pdb.set_trace()
        R_error = np.degrees(np.arccos(1- np.abs((1-(0.5*(np.trace(np.matmul(np.linalg.inv(R_est),R_GT))-1))))))
    List_err_T.append(T_error)
    List_err_R.append(R_error)

mean_error_R = np.mean(List_err_R) #degree
mean_error_T = np.mean(List_err_T) #meter
mean_error_fx = np.mean(List_err_fx) 
mean_error_fy = np.mean(List_err_fy) 
mean_error_u0 = np.mean(List_err_u0) 
mean_error_v0 = np.mean(List_err_v0) 
print('error translation (meter) : ', mean_error_T)
print('error rtotation (degree) : ', mean_error_R)
print('error fx (px) : ', mean_error_fx)
print('error fy (px) : ', mean_error_fy)
print('error u0 (px) : ', mean_error_u0)
print('error v0 (px) : ', mean_error_v0)
print('error pp (px) : ', (mean_error_v0 + mean_error_u0)/2)

axisEqual3D(ax)
#ax.grid(False)
ax.view_init(elev=0., azim=-90)
ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.savefig('4-Unbalanced.pdf',format='pdf')
plt.show()


 
