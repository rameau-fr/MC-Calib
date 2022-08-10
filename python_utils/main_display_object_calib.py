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

#!!!! This display code assumes a single objects remain at the end the calibration process
# 1. path to object calibration results
#path_root = "/home/francois/Documents/CppProject/Stereo_Calibration/Images_Synth_4cam_8boards_4groups_NonOverlap/"
path_root = "/disk/francois/Projets/Calibration_toolbox/Data/Real_Images/stereo_cube/"
path_root = "/disk/francois/Projets/Calibration_toolbox/Data/Real_Images/2021-07-12-15-09-53/"
path_object_results = path_root + "calibrated_objects_data.yml"

marker_size = 1

# 2. Open the file
fs = cv2.FileStorage(path_object_results, cv2.FILE_STORAGE_READ)
obj_id = "object_" + str(0)
obj_mat = fs.getNode(obj_id).getNode("points").mat()

#2. plot
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_proj_type('ortho')
ax.set_title('3D calibrated object')
#ax.scatter(obj_mat[0,:], obj_mat[1,:], obj_mat[2,:], c=(1.0,0.0,0.0))

#To display the cube with different color on each side
val = 0
ax.scatter(obj_mat[0,val:val+16], obj_mat[1,val:val+16], obj_mat[2,val:val+16], c=(1.0,0.0,0.0))
val = val + 16
ax.scatter(obj_mat[0,val:val+16], obj_mat[1,val:val+16], obj_mat[2,val:val+16], c=(0.0,1.0,0.0))
val = val + 16
ax.scatter(obj_mat[0,val:val+16], obj_mat[1,val:val+16], obj_mat[2,val:val+16], c=(0.0,0.0,1.0))
val = val + 16
ax.scatter(obj_mat[0,val:val+16], obj_mat[1,val:val+16], obj_mat[2,val:val+16], c=(1.0,0.0,1.0))
val = val + 16
ax.scatter(obj_mat[0,val:val+16], obj_mat[1,val:val+16], obj_mat[2,val:val+16], c=(0.0,1.0,1.0))
val = val + 16
ax.scatter(obj_mat[0,val:val+16], obj_mat[1,val:val+16], obj_mat[2,val:val+16], c=(0.0,0.0,0.0))

#ax.axis('equal')
#ax.set_aspect('equal', 'box')
#fig.tight_layout()
ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
axisEqual3D(ax)
ax.set_xlabel('x (cm)',fontsize=20)
ax.set_ylabel('y (cm)',fontsize=20)
ax.set_zlabel('z (cm)',fontsize=20)
plt.show()

# 2. Display the cameras
