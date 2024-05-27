import pdb
import random

import cv2
import matplotlib.pyplot as plt
import numpy as np

#!!!! This display code assumes a single camera group remain at the end the calibration process
# 1. path to object calibration results
path_reprojection_error = "/home/francois/Documents/CppProject/Stereo_Calibration/Images_NonOver6Cam/reprojection_error_data.yml"
Nb_Camera = 1000

# Generate one color randomly per camera
camera_color = []
for i in range(0, Nb_Camera):
    c = (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
    camera_color.append(c)


# 2. Open the file
fs = cv2.FileStorage(path_reprojection_error, cv2.FILE_STORAGE_READ)
Nb_cameragroup = fs.getNode("nb_camera_group").real()
Nb_cameragroup = int(Nb_cameragroup)
camera_group_id = "camera_group_" + str(0)
frame_list = fs.getNode(camera_group_id).getNode("frame_list").mat()
list_mean_error = []
list_color = []
list_mean_error_frame = []
for i in range(0, frame_list.shape[0]):
    frame_id = "frame_" + str(frame_list[i][0])
    camera_list = (
        fs.getNode(camera_group_id).getNode(frame_id).getNode("camera_list").mat()
    )
    errors_current_frame = []

    for j in range(0, camera_list.shape[0]):
        camera_id = "camera_" + str(camera_list[j][0])
        error_pts = (
            fs.getNode(camera_group_id)
            .getNode(frame_id)
            .getNode(camera_id)
            .getNode("error_list")
            .mat()
        )
        mean_err = np.mean(error_pts)
        list_mean_error.append(mean_err)
        list_color.append(camera_color[camera_list[j][0]])
        errors_current_frame.append(mean_err)

    list_mean_error_frame.append(np.mean(errors_current_frame))
    # after each frame include enpty value for display
    for k in range(0, 5):
        list_mean_error.append(0)
        list_color.append(camera_color[camera_list[0][0]])


barlist = plt.bar(range(0, len(list_mean_error_frame)), list_mean_error_frame)
plt.xlabel("frame")
plt.ylabel("Mean reprojection error")
plt.title("Mean error per frame")
plt.show()
"""
barlist = plt.bar(range(0,len(list_mean_error)),list_mean_error); 
for i in range(0,len(list_mean_error)):
    barlist[i].set_color(list_color[i])
plt.show()
"""
