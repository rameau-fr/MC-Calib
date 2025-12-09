"""
Part 1.3 – Validation & Visualization
-------------------------------------

This script loads calibration results from `calibration.main_real()`,
computes reprojection error, visualizes the reprojected corners over images,
and writes:

    results/calibration_params.json
    results/poses.json
    results/metrics.json
    results/visualizations/*.png

"""

import os
import json
import cv2
import numpy as np
from typing import Dict, List, Tuple

from calibration import (
    load_coco_calibration,
    unpack_params,
    build_checkerboard_points,
    main_real,
    ds_project,
    kb_project,
)


project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


# ============================================================================
# Utility: ensure results folder exists
# ============================================================================
def ensure_dir(path: str):
    if not os.path.exists(path):
        os.makedirs(path)


# ============================================================================
# Draw reprojected vs observed points
# ============================================================================
def draw_reprojection(image, uv_obs, uv_proj, valid_mask):
    """
    Draws:
        - observed 2D points in BLUE
        - reprojected points in RED
    """
    img = image.copy()

    for (uo, vo), (up, vp), v in zip(uv_obs, uv_proj, valid_mask):
        if not v:
            continue

        cv2.circle(img, (int(uo), int(vo)), 5, (255, 0, 0), -1)  # observed (blue)
        cv2.circle(img, (int(up), int(vp)), 4, (0, 0, 255), -1)  # projected (red)
        cv2.line(img, (int(uo), int(vo)), (int(up), int(vp)), (0, 255, 0), 2)

    return img


# ============================================================================
# Compute per-image RMS error
# ============================================================================
def compute_rms_error(uv_proj, uv_obs, valid_mask):
    diff = uv_proj[valid_mask] - uv_obs[valid_mask]
    if diff.size == 0:
        return 0.0
    return np.sqrt(np.mean(np.sum(diff**2, axis=1)))


# ============================================================================
# MAIN VALIDATION PIPELINE
# ============================================================================
def main_validate():
    """
    Validation pipeline.

    Loads:
        - results/calibration_params.json
        - results/poses.json

    Computes:
        - Per-image reprojection visualizations
        - Per-image RMS, global statistics

    Saves:
        results/visualizations/*.png
        results/metrics.json
    """

    # --------------------------------------------------------------
    # 1. Load saved calibration parameters & poses
    # --------------------------------------------------------------
    with open("results/calibration_params.json", "r") as f:
        calib = json.load(f)

    with open("results/poses.json", "r") as f:
        poses = json.load(f)

    # intrinsics + DS/KB
    model_type = calib.get("model_type", "ds")
    fx = calib["fx"]
    fy = calib["fy"]
    cx = calib["cx"]
    cy = calib["cy"]

    if model_type == "ds":
        xi = calib["xi"]
        alpha = calib["alpha"]
    else:
        k1 = calib["k1"]
        k2 = calib["k2"]
        k3 = calib["k3"]
        k4 = calib["k4"]

    ph = calib["checkerboard_rows"]
    pw = calib["checkerboard_cols"]
    pLength = calib["pLength"]

    # --------------------------------------------------------------
    # 2. Load original COCO annotation for ground-truth keypoints
    # --------------------------------------------------------------
    
    json_path = project_root + "/anns.json"

    (
        X_world_list,
        keypoints_list,
        visibility_list,
        image_info_list
    ) = load_coco_calibration(json_path, ph, pw, pLength)

    num_images = len(image_info_list)

    # --------------------------------------------------------------
    # 3. Prepare output directories
    # --------------------------------------------------------------
    out_dir = "results"
    vis_dir = os.path.join(out_dir, "visualizations")

    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(vis_dir, exist_ok=True)

    # --------------------------------------------------------------
    # 4. Loop through each image and compute projection error
    # --------------------------------------------------------------
    all_rms = []

    for i in range(num_images):

        img_rel = image_info_list[i]["file_name"]     # e.g., samples/000011.jpg
        # Always resolve paths relative to project root
        img_path = os.path.join(project_root, img_rel)


        img = cv2.imread(img_path)
        if img is None:
            print(f"WARNING: Could not load image {img_path}")
            continue

        # 2D annotations
        uv_obs = keypoints_list[i]
        vis = visibility_list[i]

        # 3D board points
        Xw = X_world_list[i]

        # Extrinsics from saved JSON
        pose = poses[img_rel]
        r_i = np.array(pose["r"], dtype=np.float64)
        t_i = np.array(pose["t"], dtype=np.float64)

        # world → camera
        R_i, _ = cv2.Rodrigues(r_i)
        Xc = (R_i @ Xw.T).T + t_i.reshape(1, 3)

        # project using selected model
        if model_type == "ds":
            uv_proj, valid_proj = ds_project(Xc, fx, fy, cx, cy, xi, alpha)
        else:
            uv_proj, valid_proj = kb_project(Xc, fx, fy, cx, cy, k1, k2, k3, k4)
        
        valid_mask = vis & valid_proj

        # RMS error
        rms = compute_rms_error(uv_proj, uv_obs, valid_mask)
        all_rms.append(rms)

        # Visualization
        vis_img = draw_reprojection(img, uv_obs, uv_proj, valid_mask)
        out_file = os.path.join(vis_dir, f"reproj_{i:03d}.png")
        cv2.imwrite(out_file, vis_img)

        print(f"Saved visualization {out_file} (RMS={rms:.4f})")

    # --------------------------------------------------------------
    # 5. Save metrics.json
    # --------------------------------------------------------------
    metrics = {
        "num_images": num_images,
        "mean_rms": float(np.mean(all_rms)),
        "median_rms": float(np.median(all_rms)),
        "max_rms": float(np.max(all_rms)),
        "min_rms": float(np.min(all_rms)),
        "per_image_rms": [float(v) for v in all_rms],
    }

    with open(os.path.join(out_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=4)

    print("\nValidation complete!")
    print("Saved:")
    print("  results/metrics.json")
    print("  results/visualizations/*.png")

    return metrics



# ============================================================================
# Run directly
# ============================================================================
if __name__ == "__main__":
    main_validate()
