MC-Calib comes with several utility scripts to investigate camera calibration results


- `compute_pose_error_vs_gt.py`

    ```bash
    usage: Display calibrated cameras vs ground truth [-h]
                                                    [--calibrated_cameras_data CALIBRATED_CAMERAS_DATA]
                                                    [--calibrated_cameras_gt CALIBRATED_CAMERAS_GT]

    Visualize and save the figures of calibrated and ground truth cameras.This display code assumes a
    single camera group remainat the end the calibration process.

    optional arguments:
    -h, --help            show this help message and exit
    --calibrated_cameras_data CALIBRATED_CAMERAS_DATA, -d CALIBRATED_CAMERAS_DATA
                            Path to calibrated_cameras_data.yml
    --calibrated_cameras_gt CALIBRATED_CAMERAS_GT, -g CALIBRATED_CAMERAS_GT
                            Path to GroundTruth.yml
    ```

- `display_cam_obj_pose.py`

    ```bash
    usage: Display calibrated camera and objects [-h] [--calib_data CALIB_DATA]

    Visualize and save the figures of calibrated cameras and objects.This display code assumes a single
    camera group remain at the end the calibration process

    optional arguments:
    -h, --help            show this help message and exit
    --calib_data CALIB_DATA, -d CALIB_DATA
                            Path to folder containing
                            calibrated_cameras_data.yml,calibrated_objects_data.yml, and
                            calibrated_objects_pose_data.yml
    ```

- `display_calib_cameras.py`

    ```bash
    usage: Display calibrated cameras [-h] [--calib_cameras_data_path CALIB_CAMERAS_DATA_PATH]

    Visualize and save the figures of calibrated cameras.This display code assumes a single camera group
    remain at the end the calibration process.

    optional arguments:
    -h, --help            show this help message and exit
    --calib_cameras_data_path CALIB_CAMERAS_DATA_PATH, -d CALIB_CAMERAS_DATA_PATH
                            Path to calibrated_cameras_data.yml
    ```

- `display_calib_object.py`

    ```bash
    usage: Display calibrated object [-h] [--calib_object_data_path CALIB_OBJECT_DATA_PATH]

    Visualize and save the figures of calibrated object.This display code assumes a single objects remain
    at the end the calibration process.

    optional arguments:
    -h, --help            show this help message and exit
    --calib_object_data_path CALIB_OBJECT_DATA_PATH, -d CALIB_OBJECT_DATA_PATH
                            Path to calibrated_objects_data.yml
    ```

- Compute and visualize mean reprojection error per frame `compute_error_statistic.py`

    ```bash
    usage: Compute error statistic [-h] [--reprojection_error_data_path REPROJECTION_ERROR_DATA_PATH]

    Compute error statistic per frame and visualize and save the figures.This display code assumes a
    single camera group remainsat the end the calibration process

    optional arguments:
    -h, --help            show this help message and exit
    --reprojection_error_data_path REPROJECTION_ERROR_DATA_PATH, -d REPROJECTION_ERROR_DATA_PATH
                            Path to reprojection_error_data.yml
    ```

- single script `post_calibration_analysis.py` combines all previous scripts:

    ```bash
    python3 post_calibration_analysis.py -d /home/MC-Calib/data/Blender_Images/Scenario_1/Results/ /home/MC-Calib/data/Blender_Images/Scenario_2/Results/ /home/MC-Calib/data/Blender_Images/Scenario_3/Results/ /home/MC-Calib/data/Blender_Images/Scenario_4/Results/ /home/MC-Calib/data/Blender_Images/Scenario_5/Results/
    ```