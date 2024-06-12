import argparse
from pathlib import Path
from typing import List

from compute_error_statistic import compute_error_statistic
from compute_pose_error_vs_gt import compute_pose_error_vs_gt
from display_calib_cameras import display_calib_cameras
from display_calib_object import display_calib_object
from display_cam_obj_pose import display_cam_obj_pose


def post_calibration_analysis(calib_data: List[Path]) -> None:
    for sequence_path in calib_data:

        assert sequence_path.exists()

        calib_cameras_data = sequence_path / "calibrated_cameras_data.yml"
        calib_cameras_data_gt = sequence_path.parent / "GroundTruth.yml"
        calib_object_data = sequence_path / "calibrated_objects_data.yml"
        reprojection_error_data = sequence_path / "reprojection_error_data.yml"

        assert calib_cameras_data.exists()
        assert calib_cameras_data_gt.exists()
        assert calib_object_data.exists()
        assert reprojection_error_data.exists()

        compute_pose_error_vs_gt(
            calibrated_cameras_data=calib_cameras_data, calibrated_cameras_gt=calib_cameras_data_gt
        )
        display_cam_obj_pose(calib_data=sequence_path)
        display_calib_cameras(calib_cameras_data_path=calib_cameras_data)
        display_calib_object(calib_object_data_path=calib_object_data)
        compute_error_statistic(reprojection_error_data_path=reprojection_error_data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Post-calibration analysis",
    )
    parser.add_argument(
        "--calib_data",
        "-d",
        nargs="+",
        type=Path,
        help="Path to calibration outputs i.e. Results",
    )
    args = parser.parse_args()
    post_calibration_analysis(calib_data=args.calib_data)
