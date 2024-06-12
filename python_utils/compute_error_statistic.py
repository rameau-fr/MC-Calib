import argparse
import random
from pathlib import Path
from typing import List, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np


def generate_color_per_camera(
    num_cameras: int = 100,
) -> List[Tuple[float, float, float]]:
    # generate one color randomly per camera
    camera_color = []
    for _ in range(num_cameras):
        c = (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        camera_color.append(c)
    return camera_color


def verify_num_camera_groups(fs: cv2.FileStorage) -> None:
    num_camera_groups = int(fs.getNode("nb_camera_group").real())
    assert (
        num_camera_groups == 1
    ), f"Number of camera groups at the end of calibration is {num_camera_groups}"


def visualize_and_save_results(
    list_mean_error_frame: List[float], reprojection_error_data_path: Path
) -> None:
    plt.bar(range(0, len(list_mean_error_frame)), list_mean_error_frame)
    plt.xlabel("frame")
    plt.ylabel("Mean reprojection error")
    plt.title("Mean error per frame")
    plt.show()
    plt.savefig(reprojection_error_data_path / "mean_reprojection_error_per_frame.png")
    plt.close()


def compute_error_statistic(reprojection_error_data_path: Path) -> None:
    camera_color = generate_color_per_camera()

    fs = cv2.FileStorage(str(reprojection_error_data_path), cv2.FILE_STORAGE_READ)
    verify_num_camera_groups(fs)

    camera_group_id = "camera_group_" + str(0)
    frame_list = fs.getNode(camera_group_id).getNode("frame_list").mat()
    list_mean_error: List[float] = []
    list_color = []
    list_mean_error_frame: List[float] = []
    for i in range(frame_list.shape[0]):
        frame_id = "frame_" + str(frame_list[i][0])
        camera_list = fs.getNode(camera_group_id).getNode(frame_id).getNode("camera_list").mat()
        errors_current_frame = []

        for j in range(camera_list.shape[0]):
            camera_id = "camera_" + str(camera_list[j][0])
            error_pts = (
                fs.getNode(camera_group_id)
                .getNode(frame_id)
                .getNode(camera_id)
                .getNode("error_list")
                .mat()
            )
            mean_err = float(np.mean(np.squeeze(error_pts)))
            list_mean_error.append(mean_err)
            list_color.append(camera_color[camera_list[j][0]])
            errors_current_frame.append(mean_err)

        list_mean_error_frame.append(float(np.mean(errors_current_frame)))
        # after each frame include empty value for display
        for _ in range(5):
            list_mean_error.append(0.0)
            list_color.append(camera_color[camera_list[0][0]])

    visualize_and_save_results(list_mean_error_frame, reprojection_error_data_path.parent)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Compute error statistic",
        description="Compute error statistic per frame and visualize and save the figures."
        "This display code assumes a single camera group remains"
        "at the end the calibration process",
    )
    parser.add_argument(
        "--reprojection_error_data_path",
        "-d",
        type=Path,
        help="Path to reprojection_error_data.yml",
    )

    args = parser.parse_args()
    compute_error_statistic(reprojection_error_data_path=args.reprojection_error_data_path)
