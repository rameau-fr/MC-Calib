MC-Calib comes with several utility scripts to investigate camera calibration results

```
# TODO add to docker environment

RUN apt update
RUN apt install python-is-python3 python3-pip
RUN pip install -r requirements_dev.txt
RUN pip install -r requirements_prod.txt
```

```bash
python3 compute_pose_error_vs_gt.py -d /home/MC-Calib/data/Blender_Images/Scenario_1/Results/calibrated_cameras_data.yml -g /home/MC-Calib/data/Blender_Images/Scenario_1/GroundTruth.yml
python3 compute_pose_error_vs_gt.py -d /home/MC-Calib/data/Blender_Images/Scenario_2/Results/calibrated_cameras_data.yml -g /home/MC-Calib/data/Blender_Images/Scenario_2/GroundTruth.yml
python3 compute_pose_error_vs_gt.py -d /home/MC-Calib/data/Blender_Images/Scenario_3/Results/calibrated_cameras_data.yml -g /home/MC-Calib/data/Blender_Images/Scenario_3/GroundTruth.yml
python3 compute_pose_error_vs_gt.py -d /home/MC-Calib/data/Blender_Images/Scenario_4/Results/calibrated_cameras_data.yml -g /home/MC-Calib/data/Blender_Images/Scenario_4/GroundTruth.yml
python3 compute_pose_error_vs_gt.py -d /home/MC-Calib/data/Blender_Images/Scenario_5/Results/calibrated_cameras_data.yml -g /home/MC-Calib/data/Blender_Images/Scenario_5/GroundTruth.yml
```

```bash
python3 display_cam_obj_pose.py -d /home/MC-Calib/data/Blender_Images/Scenario_1/Results
python3 display_cam_obj_pose.py -d /home/MC-Calib/data/Blender_Images/Scenario_2/Results
python3 display_cam_obj_pose.py -d /home/MC-Calib/data/Blender_Images/Scenario_3/Results
python3 display_cam_obj_pose.py -d /home/MC-Calib/data/Blender_Images/Scenario_4/Results
python3 display_cam_obj_pose.py -d /home/MC-Calib/data/Blender_Images/Scenario_5/Results
```

```bash
python3 display_calib_cameras.py -d /home/MC-Calib/data/Blender_Images/Scenario_1/Results/calibrated_cameras_data.yml
python3 display_calib_cameras.py -d /home/MC-Calib/data/Blender_Images/Scenario_2/Results/calibrated_cameras_data.yml
python3 display_calib_cameras.py -d /home/MC-Calib/data/Blender_Images/Scenario_3/Results/calibrated_cameras_data.yml
python3 display_calib_cameras.py -d /home/MC-Calib/data/Blender_Images/Scenario_4/Results/calibrated_cameras_data.yml
python3 display_calib_cameras.py -d /home/MC-Calib/data/Blender_Images/Scenario_5/Results/calibrated_cameras_data.yml
```

```bash
python display_calib_object.py -d /home/MC-Calib/data/Blender_Images/Scenario_1/Results/calibrated_objects_data.yml
python display_calib_object.py -d /home/MC-Calib/data/Blender_Images/Scenario_2/Results/calibrated_objects_data.yml
python display_calib_object.py -d /home/MC-Calib/data/Blender_Images/Scenario_3/Results/calibrated_objects_data.yml
python display_calib_object.py -d /home/MC-Calib/data/Blender_Images/Scenario_4/Results/calibrated_objects_data.yml
python display_calib_object.py -d /home/MC-Calib/data/Blender_Images/Scenario_5/Results/calibrated_objects_data.yml
```

Compute and visualize mean reprojection error per frame

```bash
python3 compute_error_statistic.py -d /home/MC-Calib/data/Blender_Images/Scenario_1/Results/reprojection_error_data.yml
python3 compute_error_statistic.py -d /home/MC-Calib/data/Blender_Images/Scenario_2/Results/reprojection_error_data.yml
python3 compute_error_statistic.py -d /home/MC-Calib/data/Blender_Images/Scenario_3/Results/reprojection_error_data.yml
python3 compute_error_statistic.py -d /home/MC-Calib/data/Blender_Images/Scenario_4/Results/reprojection_error_data.yml
python3 compute_error_statistic.py -d /home/MC-Calib/data/Blender_Images/Scenario_5/Results/reprojection_error_data.yml
```