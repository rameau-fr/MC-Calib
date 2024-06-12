import argparse
from pathlib import Path
from typing import List, Optional

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes

from utils import prepare_figure


def plot_camera(
    ax: Axes, coords: List[np.ndarray], cam_color: List[float], cam_edge: float
) -> None:
    assert len(coords) == 16

    x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16 = coords

    ax.plot(
        [x1[0][0], x2[0][0]],
        [x1[1][0], x2[1][0]],
        zs=[x1[2][0], x2[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x1[0][0], x3[0][0]],
        [x1[1][0], x3[1][0]],
        zs=[x1[2][0], x3[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x4[0][0], x3[0][0]],
        [x4[1][0], x3[1][0]],
        zs=[x4[2][0], x4[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x2[0][0], x4[0][0]],
        [x2[1][0], x4[1][0]],
        zs=[x2[2][0], x4[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )

    ax.plot(
        [x5[0][0], x6[0][0]],
        [x5[1][0], x6[1][0]],
        zs=[x5[2][0], x6[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x5[0][0], x7[0][0]],
        [x5[1][0], x7[1][0]],
        zs=[x5[2][0], x7[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x8[0][0], x7[0][0]],
        [x8[1][0], x7[1][0]],
        zs=[x8[2][0], x7[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x6[0][0], x8[0][0]],
        [x6[1][0], x8[1][0]],
        zs=[x6[2][0], x8[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )

    ax.plot(
        [x1[0][0], x5[0][0]],
        [x1[1][0], x5[1][0]],
        zs=[x1[2][0], x5[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x2[0][0], x6[0][0]],
        [x2[1][0], x6[1][0]],
        zs=[x2[2][0], x6[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x3[0][0], x7[0][0]],
        [x3[1][0], x7[1][0]],
        zs=[x3[2][0], x7[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x4[0][0], x8[0][0]],
        [x4[1][0], x8[1][0]],
        zs=[x4[2][0], x8[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )

    ax.plot(
        [x9[0][0], x10[0][0]],
        [x9[1][0], x10[1][0]],
        zs=[x9[2][0], x10[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x9[0][0], x11[0][0]],
        [x9[1][0], x11[1][0]],
        zs=[x9[2][0], x11[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x12[0][0], x11[0][0]],
        [x12[1][0], x11[1][0]],
        zs=[x12[2][0], x11[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x10[0][0], x12[0][0]],
        [x10[1][0], x12[1][0]],
        zs=[x10[2][0], x12[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )

    ax.plot(
        [x5[0][0], x9[0][0]],
        [x5[1][0], x9[1][0]],
        zs=[x5[2][0], x9[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x6[0][0], x10[0][0]],
        [x6[1][0], x10[1][0]],
        zs=[x6[2][0], x10[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x7[0][0], x11[0][0]],
        [x7[1][0], x11[1][0]],
        zs=[x7[2][0], x11[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )
    ax.plot(
        [x8[0][0], x12[0][0]],
        [x8[1][0], x8[1][0]],
        zs=[x8[2][0], x12[2][0]],
        color=(cam_color[0], cam_color[1], cam_color[2]),
        linewidth=cam_edge,
    )

    ax.plot(
        [x13[0][0], x14[0][0]],
        [x13[1][0], x14[1][0]],
        zs=[x13[2][0], x14[2][0]],
        color=(1.0, 0.0, 0.0),
        linewidth=cam_edge,
    )
    ax.plot(
        [x13[0][0], x15[0][0]],
        [x13[1][0], x15[1][0]],
        zs=[x13[2][0], x15[2][0]],
        color=(0.0, 1.0, 0.0),
        linewidth=cam_edge,
    )
    ax.plot(
        [x13[0][0], x16[0][0]],
        [x13[1][0], x16[1][0]],
        zs=[x13[2][0], x16[2][0]],
        color=(0.0, 0.0, 1.0),
        linewidth=cam_edge,
    )


def add_camera_to_subplot(
    ax: Axes,
    cam_rot: np.ndarray,
    cam_trans: np.ndarray,
    cam_size: float,
    cam_edge: float,
    cam_color: Optional[List[float]] = None,
) -> None:
    # CameraSize: half of the camera body length
    # CameraEdge: line width of the camera body
    # CameraColor: 3-vector RGB color of the camera

    if not cam_color:
        cam_color = [0.0, 0.0, 0.0]

    r = cam_size

    # Corners of the camera in the camera coordinate system
    x1 = np.asarray([[r, -r, -2 * r]]).T
    x2 = np.asarray([[r, r, -2 * r]]).T
    x3 = np.asarray([[-r, -r, -2 * r]]).T
    x4 = np.asarray([[-r, r, -2 * r]]).T

    x5 = np.asarray([[r, -r, 2 * r]]).T
    x6 = np.asarray([[r, r, 2 * r]]).T
    x7 = np.asarray([[-r, -r, 2 * r]]).T
    x8 = np.asarray([[-r, r, 2 * r]]).T

    x9 = np.asarray([[1.5 * r, -1.5 * r, 3 * r]]).T
    x10 = np.asarray([[1.5 * r, 1.5 * r, 3 * r]]).T
    x11 = np.asarray([[-1.5 * r, -1.5 * r, 3 * r]]).T
    x12 = np.asarray([[-1.5 * r, 1.5 * r, 3 * r]]).T

    x13 = np.asarray([[0, 0, 0]]).T
    x14 = np.asarray([[0, 0, 2 * r]]).T
    x15 = np.asarray([[0, 2 * r, 0]]).T
    x16 = np.asarray([[2 * r, 0, 0]]).T

    # Corners of the camera in the world coordinate system
    x1 = np.matmul(cam_rot, x1) + cam_trans
    x2 = np.matmul(cam_rot, x2) + cam_trans
    x3 = np.matmul(cam_rot, x3) + cam_trans
    x4 = np.matmul(cam_rot, x4) + cam_trans
    x5 = np.matmul(cam_rot, x5) + cam_trans
    x6 = np.matmul(cam_rot, x6) + cam_trans
    x7 = np.matmul(cam_rot, x7) + cam_trans
    x8 = np.matmul(cam_rot, x8) + cam_trans
    x9 = np.matmul(cam_rot, x9) + cam_trans
    x10 = np.matmul(cam_rot, x10) + cam_trans
    x11 = np.matmul(cam_rot, x11) + cam_trans
    x12 = np.matmul(cam_rot, x12) + cam_trans
    x13 = np.matmul(cam_rot, x13) + cam_trans
    x14 = np.matmul(cam_rot, x14) + cam_trans
    x15 = np.matmul(cam_rot, x15) + cam_trans
    x16 = np.matmul(cam_rot, x16) + cam_trans

    coords = [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16]

    plot_camera(ax, coords, cam_color, cam_edge)


def display_calib_cameras(calib_cameras_data_path: Path) -> None:
    fs = cv2.FileStorage(str(calib_cameras_data_path), cv2.FILE_STORAGE_READ)
    num_cameras = int(fs.getNode("nb_camera").real())

    fig = plt.figure()
    ax: Axes = fig.add_subplot(projection="3d")
    ax.set_proj_type("ortho")
    ax.set_title("Calibration Result")

    for cam_idx in range(num_cameras):
        cam_name: str = f"camera_{cam_idx}"
        cam_pose: np.ndarray = fs.getNode(cam_name).getNode("camera_pose_matrix").mat()
        cam_trans: np.ndarray = np.asarray([cam_pose[0:3, 3]]).T
        cam_rot: np.ndarray = cam_pose[0:3, 0:3]
        add_camera_to_subplot(ax, cam_rot, cam_trans, cam_size=0.5, cam_edge=2)
        print(cam_pose)

    prepare_figure(ax)
    plt.show()
    plt.savefig(calib_cameras_data_path.parent / "cameras_calib.png")
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Display calibrated cameras",
        description="Visualize and save the figures of calibrated cameras."
        "This display code assumes a single camera group remain "
        "at the end the calibration process.",
    )
    parser.add_argument(
        "--calib_cameras_data_path",
        "-d",
        type=Path,
        help="Path to calibrated_cameras_data.yml",
    )

    args = parser.parse_args()
    display_calib_cameras(calib_cameras_data_path=args.calib_cameras_data_path)
