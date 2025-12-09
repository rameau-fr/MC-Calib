"""
Part 1.2: Double Sphere Camera Calibration

This module defines:
- Parameter vector layout (intrinsics + DS params + per-image extrinsics)
- Residual function for non-linear least squares calibration
- Synthetic test harness to validate the pipeline
- Real COCO-based calibration entry point

"""

from typing import List, Tuple, Dict

import json
import numpy as np
import cv2
import os
import sys
from scipy.optimize import least_squares


def ds_project(X_cam, fx, fy, cx, cy, xi, alpha):
    """
    Project 3D camera-frame points into pixel coordinates using
    the Double Sphere camera model.

    Parameters
    ----------
    X_cam : (..., 3) ndarray
        3D points in **camera coordinates**
        Convention: +x right, +y down, +z forward.
    fx, fy : float
        Focal lengths in pixels
    cx, cy : float
        Principal point
    xi, alpha : float
        Double Sphere distortion parameters

    Returns
    -------
    uv : (..., 2) ndarray
        Projected pixel coordinates
    valid : (...,) boolean ndarray
        True if the point is valid inside DS projection domain
    """

    # Unpack coordinates
    x = X_cam[..., 0]
    y = X_cam[..., 1]
    z = X_cam[..., 2]

    # Points must be in front of the camera
    valid = z > 0

    # === First sphere distance (norm of 3D point) ===
    d1 = np.sqrt(x * x + y * y + z * z)

    # === Shift along z-axis by xi (DS inner geometry step) ===
    z1 = z + xi * d1

    # === Second sphere distance ===
    d2 = np.sqrt(x * x + y * y + z1 * z1)

    # === Denominator for projection ===
    #    α * d2 + (1 - α) * z1
    den = alpha * d2 + (1.0 - alpha) * z1

    # Avoid division by zero
    valid &= den > 1e-8
    den = np.maximum(den, 1e-8)

    # === Actual Projection ===
    u = fx * x / den + cx
    v = fy * y / den + cy

    uv = np.stack([u, v], axis=-1)
    return uv, valid


def kb_project(X_cam, fx, fy, cx, cy, k1, k2, k3, k4):
    """
    Project 3D camera-frame points using the Kannala-Brandt (OpenCV fisheye) model.
    """
    x = X_cam[..., 0]
    y = X_cam[..., 1]
    z = X_cam[..., 2]

    valid = z > 0
    r = np.sqrt(x * x + y * y)
    theta = np.arctan2(r, z)

    theta2 = theta * theta
    theta4 = theta2 * theta2
    theta6 = theta4 * theta2
    theta8 = theta4 * theta4

    theta_d = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8)

    # Avoid division by zero
    scale = np.zeros_like(r)
    mask = r > 1e-8
    scale[mask] = theta_d[mask] / r[mask]
    scale[~mask] = 1.0

    u = fx * x * scale + cx
    v = fy * y * scale + cy

    return np.stack([u, v], axis=-1), valid


def ds_unproject(uv, fx, fy, cx, cy, xi, alpha):
    """
    Double Sphere back-projection (Usenko et al., 2018)
    Converts a pixel (u, v) to a unit 3D ray in the camera frame.

    Parameters
    ----------
    uv : (..., 2) ndarray
        Pixel coordinates
    fx, fy, cx, cy : float
        Intrinsic parameters
    xi, alpha : float
        DS distortion parameters

    Returns
    -------
    ray : (..., 3) ndarray
        Unit 3D direction vectors in camera coordinates
    valid : (...,) ndarray
        Validity mask
    """

    u = uv[..., 0]
    v = uv[..., 1]

    # Normalize to metric camera coordinates
    mx = (u - cx) / fx
    my = (v - cy) / fy
    r2 = mx * mx + my * my

    # Validity check (from the paper)
    s = 1.0 - (2.0 * alpha - 1.0) * r2
    valid = s >= 0
    s = np.maximum(s, 0.0)

    # Compute mz using DS formula
    mz = (1.0 - alpha * alpha * r2) / (alpha * np.sqrt(s) + (1.0 - alpha))

    # Compute denominator for scaling
    mz2 = mz * mz
    k1 = mz * xi + np.sqrt(mz2 + (1.0 - xi * xi) * r2)
    k2 = mz2 + r2
    k = k1 / k2

    # Form the ray direction (before normalization)
    X = k * mx
    Y = k * my
    Z = k * mz - xi

    ray = np.stack([X, Y, Z], axis=-1)

    # Normalize (should already be close to unit)
    norm = np.linalg.norm(ray, axis=-1, keepdims=True)
    ray /= norm + 1e-8

    return ray, valid


# ================================================================
# 3D checkerboard geometry
# ================================================================


def build_checkerboard_points(ph: int, pw: int, pLength: float) -> np.ndarray:
    """
    Build 3D world coordinates for a ph x pw checkerboard.

    World frame:
        - Origin at top-left corner
        - X axis along width (columns)
        - Y axis along height (rows)
        - Z = 0 (board is planar)

    The ordering is row-major: first row (y=0), x=0..pw-1, then next row, etc.
    This must match the keypoint ordering in the COCO annotations.

    Parameters
    ----------
    ph : int
        Number of checkerboard corners in vertical direction (rows).
    pw : int
        Number of checkerboard corners in horizontal direction (columns).
    pLength : float
        Physical spacing between neighboring corners (same units you want
        for translation, e.g. meters or centimeters).

    Returns
    -------
    Xw : (ph*pw, 3) ndarray
        3D world points.
    """
    xs = np.arange(pw) * pLength
    ys = np.arange(ph) * pLength
    X, Y = np.meshgrid(xs, ys, indexing="xy")
    Z = np.zeros_like(X)

    Xw = np.stack([X, Y, Z], axis=-1)  # (ph, pw, 3)
    return Xw.reshape(-1, 3)  # (ph*pw, 3)


# ================================================================
# Parameter packing / unpacking
# ================================================================


def pack_params(
    intrinsics: np.ndarray,
    r_list: List[np.ndarray],
    t_list: List[np.ndarray],
) -> np.ndarray:
    """
    Pack all calibration parameters into a single 1D vector.

    Parameters
    ----------
    intrinsics : ndarray
        Intrinsic parameters (6 for DS, 8 for KB)
    r_list : list of (3,) ndarrays
    t_list : list of (3,) ndarrays
    """
    num_images = len(r_list)
    assert num_images == len(t_list), "r_list and t_list must have same length"

    # Start with intrinsics
    param_list = list(intrinsics)

    # Then 6 params per image: [r_i(3), t_i(3)]
    for i in range(num_images):
        r_i = np.asarray(r_list[i]).reshape(3)
        t_i = np.asarray(t_list[i]).reshape(3)
        param_list.extend(r_i.tolist())
        param_list.extend(t_i.tolist())

    return np.array(param_list, dtype=np.float64)


def unpack_params(
    params: np.ndarray,
    num_images: int,
    num_intrinsics: int,
) -> Tuple[np.ndarray, List[np.ndarray], List[np.ndarray]]:
    """
    Unpack 1D parameter vector.
    """
    params = np.asarray(params).ravel()
    expected_len = num_intrinsics + 6 * num_images
    assert (
        params.shape[0] == expected_len
    ), f"Expected param length {expected_len}, got {params.shape[0]}"

    # Intrinsics
    intrinsics = params[:num_intrinsics]

    r_list = []
    t_list = []

    # Each image contributes 6 params
    offset = num_intrinsics
    for _ in range(num_images):
        r_i = params[offset : offset + 3]
        t_i = params[offset + 3 : offset + 6]
        r_list.append(r_i.copy())
        t_list.append(t_i.copy())
        offset += 6

    return intrinsics, r_list, t_list


# ================================================================
# COCO-style data loader for real calibration data
# ================================================================


def load_coco_calibration(
    json_path: str,
    ph: int,
    pw: int,
    pLength: float,
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[Dict]]:
    """
    Load COCO-style calibration annotations and construct:
    - 3D checkerboard world points per image
    - 2D keypoints per image
    - visibility masks per image
    - image metadata

    Assumes:
    - Exactly one checkerboard annotation per image.
    - 'keypoints' are of length 3 * (ph * pw) and ordered row-major
      from top-left, matching build_checkerboard_points().

    Parameters
    ----------
    json_path : str
        Path to the COCO-style JSON file.
    ph, pw : int
        Checkerboard rows and columns.
    pLength : float
        Physical spacing between corners.

    Returns
    -------
    X_world_list : list of (N, 3) ndarray
    keypoints_list : list of (N, 2) ndarray
    visibility_list : list of (N,) bool ndarray
    image_info_list : list of dict
        Raw COCO 'images' entries.
    """
    with open(json_path, "r") as f:
        data = json.load(f)

    images = data["images"]
    annotations = data["annotations"]

    # Map image_id -> annotation (assuming one board per image)
    ann_by_img: Dict[int, dict] = {}
    for ann in annotations:
        img_id = ann["image_id"]
        if img_id in ann_by_img:
            raise ValueError(f"Multiple annotations for image_id={img_id}")
        ann_by_img[img_id] = ann

    Xw_board = build_checkerboard_points(ph, pw, pLength)
    N_expected = ph * pw

    X_world_list: List[np.ndarray] = []
    keypoints_list: List[np.ndarray] = []
    visibility_list: List[np.ndarray] = []
    image_info_list: List[Dict] = []

    # Iterate images in order of id
    for img in sorted(images, key=lambda im: im["id"]):
        img_id = img["id"]
        if img_id not in ann_by_img:
            # No board for this image; skip
            continue

        ann = ann_by_img[img_id]
        kps = np.array(ann["keypoints"], dtype=np.float32).reshape(-1, 3)
        if kps.shape[0] != N_expected:
            raise ValueError(
                f"Image {img_id}: expected {N_expected} keypoints, got {kps.shape[0]}"
            )

        uv = kps[:, :2]  # (N, 2)
        vis = kps[:, 2] == 2  # (N,)

        X_world_list.append(Xw_board.copy())
        keypoints_list.append(uv)
        visibility_list.append(vis)
        image_info_list.append(img)

    return X_world_list, keypoints_list, visibility_list, image_info_list


# ================================================================
# Residual function for least squares
# ================================================================


def calibration_residuals(
    params: np.ndarray,
    X_world_list: List[np.ndarray],
    keypoints_list: List[np.ndarray],
    visibility_list: List[np.ndarray],
    model_type: str = "ds",
) -> np.ndarray:
    """
    Compute reprojection residuals for all images and points.
    """
    num_images = len(X_world_list)
    assert (
        num_images == len(keypoints_list) == len(visibility_list)
    ), "Inconsistent list lengths for images / keypoints / visibility"

    # Determine num_intrinsics based on model
    if model_type == "ds":
        num_intrinsics = 6
    elif model_type == "kb":
        num_intrinsics = 8
    else:
        raise ValueError(f"Unknown model_type: {model_type}")

    # Unpack global + per-image params
    intrinsics, r_list, t_list = unpack_params(
        params, num_images=num_images, num_intrinsics=num_intrinsics
    )

    if model_type == "ds":
        fx, fy, cx, cy, xi, alpha = intrinsics
    else:
        fx, fy, cx, cy, k1, k2, k3, k4 = intrinsics

    residuals_all = []

    for i in range(num_images):
        Xw = X_world_list[i]  # (N, 3)
        uv_obs = keypoints_list[i]  # (N, 2)
        vis = visibility_list[i]  # (N,)

        assert (
            Xw.shape[0] == uv_obs.shape[0] == vis.shape[0]
        ), f"Mismatch in N for image {i}"

        # --- World to camera: X_cam = R_i * X_world + t_i ---
        r_i = r_list[i]
        t_i = t_list[i]

        R_i, _ = cv2.Rodrigues(r_i.astype(np.float64))
        Xc = (R_i @ Xw.T).T + t_i.reshape(1, 3)

        # --- Project using selected model ---
        if model_type == "ds":
            uv_proj, valid_proj = ds_project(Xc, fx, fy, cx, cy, xi, alpha)
        else:
            uv_proj, valid_proj = kb_project(Xc, fx, fy, cx, cy, k1, k2, k3, k4)

        # Combine annotation visibility + projection validity
        valid = vis & valid_proj

        # Fixed-size residuals: start with zeros
        diff = np.zeros_like(uv_obs, dtype=np.float64)  # (N, 2)

        # Only where valid do we use true reprojection error
        diff[valid] = uv_proj[valid] - uv_obs[valid]

        # Append flattened residuals for this image (2N,)
        residuals_all.append(diff.reshape(-1))

    # Final residual vector: shape = (num_images * N * 2,)
    return np.concatenate(residuals_all, axis=0)


# ================================================================
# Bounds for optimization
# ================================================================


def build_bounds(
    num_images: int, img_width: float, img_height: float, model_type: str = "ds"
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Build lower/upper bounds for all parameters.
    """
    # ---- Intrinsic bounds ----
    # Focal lengths: between 500 and 4000 pixels
    fx_lb, fy_lb = 500.0, 500.0
    fx_ub, fy_ub = 4000.0, 4000.0

    # Principal point: anywhere on the image plane
    cx_lb, cy_lb = 0.0, 0.0
    cx_ub, cy_ub = img_width, img_height

    if model_type == "ds":
        # DS params:
        #   xi >= 0  (often between ~0.5 and 2.5 for fisheye)
        #   alpha in (0, 1)
        xi_lb, xi_ub = 0.1, 3.0
        alpha_lb, alpha_ub = 0.01, 0.99
        intr_lb = np.array([fx_lb, fy_lb, cx_lb, cy_lb, xi_lb, alpha_lb], dtype=np.float64)
        intr_ub = np.array([fx_ub, fy_ub, cx_ub, cy_ub, xi_ub, alpha_ub], dtype=np.float64)
    elif model_type == "kb":
        # KB params: k1, k2, k3, k4
        # Allow reasonable range for distortion coefficients
        k_lb = -10.0
        k_ub = 10.0
        intr_lb = np.array([fx_lb, fy_lb, cx_lb, cy_lb, k_lb, k_lb, k_lb, k_lb], dtype=np.float64)
        intr_ub = np.array([fx_ub, fy_ub, cx_ub, cy_ub, k_ub, k_ub, k_ub, k_ub], dtype=np.float64)

    # ---- Extrinsics bounds ----
    # Rotation as Rodrigues: each component in [-pi, pi]
    rot_lb = -np.pi * np.ones(3, dtype=np.float64)
    rot_ub = np.pi * np.ones(3, dtype=np.float64)

    # Translation: x,y in [-5, 5] meters, z in [0.2, 20]
    # (board is always in front of camera)
    tx_lb, ty_lb, tz_lb = -5.0, -5.0, 0.2
    tx_ub, ty_ub, tz_ub = 5.0, 5.0, 20.0

    trans_lb = np.array([tx_lb, ty_lb, tz_lb], dtype=np.float64)
    trans_ub = np.array([tx_ub, ty_ub, tz_ub], dtype=np.float64)

    # Pack all bounds
    lb_list = [intr_lb]
    ub_list = [intr_ub]

    for _ in range(num_images):
        lb_list.append(rot_lb)
        lb_list.append(trans_lb)
        ub_list.append(rot_ub)
        ub_list.append(trans_ub)

    lb = np.concatenate(lb_list, axis=0)
    ub = np.concatenate(ub_list, axis=0)

    return lb, ub


# ================================================================
# Real-data calibration main
# ================================================================


def initialize_kb(X_world_list, keypoints_list, image_info_list):
    """
    Initialize KB model parameters using OpenCV's fisheye calibration.
    """
    print("[KB] Initializing using cv2.fisheye.calibrate...")
    
    # Prepare data for cv2.fisheye.calibrate
    # OpenCV expects list of (N, 1, 3) and (N, 1, 2)
    obj_points = [Xw.reshape(-1, 1, 3).astype(np.float32) for Xw in X_world_list]
    img_points = [kp.reshape(-1, 1, 2).astype(np.float32) for kp in keypoints_list]
    
    h = int(image_info_list[0]['height'])
    w = int(image_info_list[0]['width'])
    
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = []
    tvecs = []
    
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    
    try:
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            obj_points, img_points, (w, h), K, D, rvecs, tvecs, flags, criteria
        )
        print(f"[KB] OpenCV Init RMS: {rms:.4f}")
    except cv2.error as e:
        print(f"[KB] OpenCV Init failed: {e}")
        # Fallback to simple init if OpenCV fails
        f0 = 0.8 * max(w, h)
        K = np.array([[f0, 0, w/2], [0, f0, h/2], [0, 0, 1]])
        D = np.zeros((4, 1))
        rvecs = [np.zeros((3, 1)) for _ in range(len(obj_points))]
        tvecs = [np.array([[0], [0], [1.5]]) for _ in range(len(obj_points))]

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    k1, k2, k3, k4 = D.flatten()
    
    return fx, fy, cx, cy, k1, k2, k3, k4, rvecs, tvecs


def main_real():
    """
    Part 1.2 – Real calibration pipeline.
    """
    # --------------------------------------------------------------
    # 0. Paths + checkerboard settings
    # --------------------------------------------------------------
    json_path = "/home/haruka_nanase/Desktop/Adv3D/CandidateInterview/Advanced_3D_Assigment/MuhammadjonBOBOEV/submission/anns.json"
    ph = 5
    pw = 6
    pLength = 0.20  # TEMP: 20 cm spacing (adjust when HR confirms)

    # Select Model: "ds" or "kb"
    model_type = "kb"  # Change this to "ds" for Double Sphere
    print(f"Running calibration with model: {model_type}")

    # --------------------------------------------------------------
    # 1. Load COCO data
    # --------------------------------------------------------------
    (X_world_list, keypoints_list, visibility_list, image_info_list) = (
        load_coco_calibration(json_path, ph, pw, pLength)
    )

    num_images = len(X_world_list)
    if num_images == 0:
        raise RuntimeError("No checkerboard annotations found.")

    img_width = float(image_info_list[0]["width"])
    img_height = float(image_info_list[0]["height"])

    # --------------------------------------------------------------
    # 2. Initial guess
    # --------------------------------------------------------------
    if model_type == "ds":
        f0 = 0.8 * max(img_width, img_height)
        fx0 = fy0 = f0
        cx0 = img_width / 2
        cy0 = img_height / 2
        xi0 = 0.5
        alpha0 = 0.5
        
        intrinsics0 = [fx0, fy0, cx0, cy0, xi0, alpha0]
        r0_list = [np.zeros(3) for _ in range(num_images)]
        t0_list = [np.array([0, 0, 1.5 + 0.01 * i]) for i in range(num_images)]
        
    elif model_type == "kb":
        fx, fy, cx, cy, k1, k2, k3, k4, rvecs, tvecs = initialize_kb(
            X_world_list, keypoints_list, image_info_list
        )
        intrinsics0 = [fx, fy, cx, cy, k1, k2, k3, k4]
        r0_list = [r.flatten() for r in rvecs]
        t0_list = [t.flatten() for t in tvecs]

    x0 = pack_params(intrinsics0, r0_list, t0_list)
    lb, ub = build_bounds(num_images, img_width, img_height, model_type)

    # --------------------------------------------------------------
    # 3. Run Levenberg–Marquardt (TRF) optimization
    # --------------------------------------------------------------
    result = least_squares(
        calibration_residuals,
        x0,
        args=(X_world_list, keypoints_list, visibility_list, model_type),
        method="trf",
        loss="linear",
        verbose=2,
        max_nfev=200,
        x_scale="jac",
        bounds=(lb, ub),
    )

    print("\n[REAL] Optimization success:", result.success)
    print("[REAL] Final cost:", result.cost)

    # --------------------------------------------------------------
    # 4. Unpack calibrated parameters
    # --------------------------------------------------------------
    num_intrinsics = 6 if model_type == "ds" else 8
    intrinsics, r_list, t_list = unpack_params(result.x, num_images, num_intrinsics)

    # RMS reprojection error (global)
    M = result.fun.shape[0]
    rms_px = np.sqrt(2 * result.cost / M)
    print(f"[REAL] RMS reprojection error: {rms_px:.4f} px")

    # --------------------------------------------------------------
    # 5. Save outputs to results/
    # --------------------------------------------------------------
    os.makedirs("results", exist_ok=True)

    # --- intrinsics + params ---
    calib_json = {
        "model_type": model_type,
        "fx": float(intrinsics[0]),
        "fy": float(intrinsics[1]),
        "cx": float(intrinsics[2]),
        "cy": float(intrinsics[3]),
        "checkerboard_rows": ph,
        "checkerboard_cols": pw,
        "pLength": pLength,
    }
    
    if model_type == "ds":
        calib_json["xi"] = float(intrinsics[4])
        calib_json["alpha"] = float(intrinsics[5])
    else:
        calib_json["k1"] = float(intrinsics[4])
        calib_json["k2"] = float(intrinsics[5])
        calib_json["k3"] = float(intrinsics[6])
        calib_json["k4"] = float(intrinsics[7])

    with open("results/calibration_params.json", "w") as f:
        json.dump(calib_json, f, indent=4)

    # --- extrinsics per image ---
    poses = {}
    for i, img in enumerate(image_info_list):
        poses[str(img["file_name"])] = {
            "r": r_list[i].tolist(),
            "t": t_list[i].tolist(),
        }
    with open("results/poses.json", "w") as f:
        json.dump(poses, f, indent=4)

    # --- metrics (global only; per-image in validate.py) ---
    metrics = {
        "global_rms_px": float(rms_px),
        "num_images": num_images,
        "model_type": model_type
    }
    with open("results/metrics.json", "w") as f:
        json.dump(metrics, f, indent=4)

    print("\nSaved:")
    print("  results/calibration_params.json")
    print("  results/poses.json")
    print("  results/metrics.json")
    print("\nRun validate.py to produce visualizations.")


if __name__ == "__main__":
    main_real()
