import argparse
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from utils import prepare_figure


def display_calib_object(calib_object_data_path: Path) -> None:
    fs = cv2.FileStorage(str(calib_object_data_path), cv2.FILE_STORAGE_READ)
    obj_id = "object_" + str(0)
    obj_mat = fs.getNode(obj_id).getNode("points").mat()

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.set_proj_type("ortho")
    ax.set_title("3D calibrated object")

    board_idx = np.unique(obj_mat[3, :]).astype(int)
    for b_idx in board_idx:
        pts_3d = obj_mat[0:3, obj_mat[3, :] == b_idx]  # gather all the points from the same board
        color = np.random.rand(3)  # random color
        ax.scatter(pts_3d[0, :], pts_3d[1, :], pts_3d[2, :], c=color.reshape(1, -1))

    prepare_figure(ax)
    plt.show()
    plt.savefig(calib_object_data_path.parent / "calibrated_object.png")
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Display calibrated object",
        description="Visualize and save the figures of calibrated object."
        "This display code assumes a single objects remain at the end the calibration process.",
    )
    parser.add_argument(
        "--calib_object_data_path",
        "-d",
        type=Path,
        help="Path to calibrated_objects_data.yml",
    )

    args = parser.parse_args()
    display_calib_object(calib_object_data_path=args.calib_object_data_path)
