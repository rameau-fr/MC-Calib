MC-Calib comes with several utility scripts to investigate camera calibration results

```
# TODO add to docker environment

RUN apt update
RUN apt install python-is-python3 python3-pip
RUN pip install -r requirements_dev.txt
RUN pip install -r requirements_prod.txt
```

```bash
python3 compute_pose_error_vs_gt.py
```

```bash
python3 display_cam_obj_pose.py
```

```bash
python3 display_cameras_calib.py
```

```bash
python display_object_calib.py
```

Compute and visualize mean reprojection error per frame

```bash
python3 compute_error_statistic.py
```