import argparse
import random
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from display_calib_cameras import add_camera_to_subplot
from utils import axis_equal_3d


def display_cam_obj_pose(calib_data: Path) -> None:
    # open and display the calibrated camera system
    path_calib_results = calib_data / "calibrated_cameras_data.yml"
    fs = cv2.FileStorage(str(path_calib_results), cv2.FILE_STORAGE_READ)
    num_cameras = int(fs.getNode("nb_camera").real())
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.set_title("Calibration Result")
    for i in range(num_cameras):
        cam_id = "camera_" + str(i)
        cam_pose = fs.getNode(cam_id).getNode("camera_pose_matrix").mat()
        cam_trans = np.asarray([cam_pose[0:3, 3]]).T
        cam_rot = cam_pose[0:3, 0:3]
        add_camera_to_subplot(ax, cam_rot, cam_trans, cam_size=0.1, cam_edge=1)
        print(cam_pose)

    # open the 3D object
    path_object_results = calib_data / "calibrated_objects_data.yml"
    fs = cv2.FileStorage(str(path_object_results), cv2.FILE_STORAGE_READ)
    obj_id = "object_" + str(0)
    obj_mat = fs.getNode(obj_id).getNode("points").mat()
    obj_pts = obj_mat[:3, :]

    # open the pose of 3D object
    path_object_results = calib_data / "calibrated_objects_pose_data.yml"
    fs = cv2.FileStorage(str(path_object_results), cv2.FILE_STORAGE_READ)
    pose_mat = fs.getNode(obj_id).getNode("poses").mat()

    step = 50
    for i in range(0, pose_mat.shape[1], step):
        rot_mat, _ = cv2.Rodrigues(pose_mat[0:3, i])
        trans_mat = pose_mat[3:, i]
        pts_3d = np.dot(obj_pts.T, rot_mat.T) + trans_mat

        color = np.array([random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)])
        ax.scatter(
            pts_3d[:, 0],
            pts_3d[:, 1],
            pts_3d[:, 2],
            c=color.reshape(1, -1),
        )

    axis_equal_3d(ax)
    plt.show()

    plt.savefig(calib_data / "cam_obj_pose.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Display calibrated camera and objects",
        description="Visualize and save the figures of calibrated cameras and objects."
        "This display code assumes a single camera group remain at the end the calibration process",
    )
    parser.add_argument(
        "--calib_data",
        "-d",
        type=Path,
        help="Path to folder containing calibrated_cameras_data.yml,"
        "calibrated_objects_data.yml, and calibrated_objects_pose_data.yml",
    )

    args = parser.parse_args()
    display_cam_obj_pose(calib_data=args.calib_data)
