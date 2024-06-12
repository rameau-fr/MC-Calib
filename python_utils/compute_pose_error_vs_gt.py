import argparse
from pathlib import Path
from typing import List, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes

from display_calib_cameras import add_camera_to_subplot
from utils import axis_equal_3d


def compute_errors(
    gt_rot_matrices: List[np.ndarray],
    gt_trans_matrices: List[np.ndarray],
    gt_intrinsic_matrices: List[np.ndarray],
    estimated_rot_matrices: List[np.ndarray],
    estimated_trans_matrices: List[np.ndarray],
    estimated_intrinsic_matrices: List[np.ndarray],
    num_cameras: int,
) -> None:
    rotation_errors: List[float] = []
    translation_errors: List[float] = []
    fx_errors: List[float] = []
    fy_errors: List[float] = []
    u0_errors: List[float] = []
    v0_errors: List[float] = []

    for i in range(num_cameras):
        gt_rot_matrix = gt_rot_matrices[i]
        est_rot_matrix = estimated_rot_matrices[i]
        gt_translation = gt_trans_matrices[i]
        est_translation = estimated_trans_matrices[i]
        gt_intrinsics = gt_intrinsic_matrices[i]
        estimated_intrinsics = estimated_intrinsic_matrices[i]
        instrinsics_error = np.sqrt((estimated_intrinsics - gt_intrinsics) ** 2)
        fx_errors.append(instrinsics_error[0][0])
        fy_errors.append(instrinsics_error[1][1])
        u0_errors.append(instrinsics_error[0][2])
        v0_errors.append(instrinsics_error[1][2])
        translation_error = np.sqrt(np.sum((gt_translation - est_translation) ** 2))
        rotation_error = np.degrees(
            np.arccos(0.5 * (np.trace(np.matmul(np.linalg.inv(est_rot_matrix), gt_rot_matrix)) - 1))
        )
        if np.isnan(rotation_error):
            rotation_error = np.degrees(
                np.arccos(
                    1
                    - np.abs(
                        (
                            1
                            - (
                                0.5
                                * (
                                    np.trace(
                                        np.matmul(np.linalg.inv(est_rot_matrix), gt_rot_matrix)
                                    )
                                    - 1
                                )
                            )
                        )
                    )
                )
            )
        translation_errors.append(translation_error)
        rotation_errors.append(rotation_error)

    mean_rot_error = np.mean(rotation_errors)  # degree
    mean_trans_error = np.mean(translation_errors)  # meter
    mean_error_fx = np.mean(fx_errors)
    mean_error_fy = np.mean(fy_errors)
    mean_error_u0 = np.mean(u0_errors)
    mean_error_v0 = np.mean(v0_errors)
    print("error translation (meter) : ", mean_trans_error)
    print("error rtotation (degree) : ", mean_rot_error)
    print("error fx (px) : ", mean_error_fx)
    print("error fy (px) : ", mean_error_fy)
    print("error u0 (px) : ", mean_error_u0)
    print("error v0 (px) : ", mean_error_v0)
    print("error pp (px) : ", (mean_error_v0 + mean_error_u0) / 2)


def read_est_results(
    calibrated_cameras_data: Path,
    ax: Axes,
    cam_size: float,
    cam_edge: float,
    cam_color: List[float],
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], int]:
    fs = cv2.FileStorage(str(calibrated_cameras_data), cv2.FILE_STORAGE_READ)
    num_est_cameras = int(fs.getNode("nb_camera").real())

    estimated_rot_matrices: List[np.ndarray] = []
    estimated_trans_matrices: List[np.ndarray] = []
    estimated_intrinsic_matrices: List[np.ndarray] = []
    for i in range(num_est_cameras):
        cam_id = "camera_" + str(i)
        cam_mat: np.ndarray = fs.getNode(cam_id).getNode("camera_matrix").mat()
        cam_pose: np.ndarray = fs.getNode(cam_id).getNode("camera_pose_matrix").mat()
        mat_trans = np.asarray([cam_pose[0:3, 3]]).T
        mat_rot = cam_pose[0:3, 0:3]
        add_camera_to_subplot(ax, mat_rot, mat_trans, cam_size, cam_edge, cam_color)
        estimated_rot_matrices.append(mat_rot)
        estimated_trans_matrices.append(mat_trans)
        estimated_intrinsic_matrices.append(cam_mat)
        print(cam_pose)

    return (
        estimated_rot_matrices,
        estimated_trans_matrices,
        estimated_intrinsic_matrices,
        num_est_cameras,
    )


def read_gt_results(
    calibrated_cameras_gt: Path,
    ax: Axes,
    cam_size: float,
    cam_edge: float,
    cam_color: List[float],
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], int]:
    fs_gt = cv2.FileStorage(str(calibrated_cameras_gt), cv2.FILE_STORAGE_READ)
    num_gt_cameras = int(fs_gt.getNode("nb_camera").real())

    gt_rot_matrices: List[np.ndarray] = []
    gt_trans_matrices: List[np.ndarray] = []
    gt_intrinsic_matrices: List[np.ndarray] = []

    for i in range(num_gt_cameras):
        transf_id = "P_" + str(i + 1)
        cam_id = "K_" + str(i + 1)
        cam_mat: np.ndarray = fs_gt.getNode(cam_id).mat()
        cam_pose: np.ndarray = fs_gt.getNode(transf_id).mat()

        trans = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        cam_pose = trans @ cam_pose @ trans
        cam_pose = np.linalg.inv(cam_pose)
        trans_matrix = np.asarray([cam_pose[0:3, 3]]).T
        rot_matrix = cam_pose[0:3, 0:3]
        add_camera_to_subplot(ax, rot_matrix, trans_matrix, cam_size, cam_edge, cam_color)
        gt_rot_matrices.append(rot_matrix)
        gt_trans_matrices.append(trans_matrix)
        gt_intrinsic_matrices.append(cam_mat)
        print(cam_pose)

    return gt_rot_matrices, gt_trans_matrices, gt_intrinsic_matrices, num_gt_cameras


def compute_pose_error_vs_gt(
    calibrated_cameras_data: Path,
    calibrated_cameras_gt: Path,
) -> None:
    cam_size = 0.5
    cam_edge = 2
    cam_color = [0.0, 0.0, 0.0]
    cam_color_gt = [1.0, 0.0, 1.0]

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    (
        estimated_rot_matrices,
        estimated_trans_matrices,
        estimated_intrinsic_matrices,
        num_est_cameras,
    ) = read_est_results(calibrated_cameras_data, ax, cam_size, cam_edge, cam_color)

    gt_rot_matrices, gt_trans_matrices, gt_intrinsic_matrices, num_gt_cameras = read_gt_results(
        calibrated_cameras_gt, ax, cam_size, cam_edge, cam_color_gt
    )

    assert num_est_cameras == num_gt_cameras

    compute_errors(
        gt_rot_matrices,
        gt_trans_matrices,
        gt_intrinsic_matrices,
        estimated_rot_matrices,
        estimated_trans_matrices,
        estimated_intrinsic_matrices,
        num_gt_cameras,
    )

    axis_equal_3d(ax)
    ax.view_init(elev=0.0, azim=-90)
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()
    plt.savefig(calibrated_cameras_data.parent / "pose_error_vs_gt.png")
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Display calibrated cameras vs ground truth",
        description="Visualize and save the figures of calibrated and ground truth cameras."
        "This display code assumes a single camera group remain"
        "at the end the calibration process.",
    )
    parser.add_argument(
        "--calibrated_cameras_data",
        "-d",
        type=Path,
        help="Path to calibrated_cameras_data.yml",
    )
    parser.add_argument(
        "--calibrated_cameras_gt",
        "-g",
        type=Path,
        help="Path to GroundTruth.yml",
    )

    args = parser.parse_args()
    compute_pose_error_vs_gt(
        calibrated_cameras_data=args.calibrated_cameras_data,
        calibrated_cameras_gt=args.calibrated_cameras_gt,
    )
