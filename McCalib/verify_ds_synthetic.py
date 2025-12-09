
import numpy as np
import cv2
import glob
import os
import sys
import json
from scipy.optimize import least_squares

# Import calibration module
# Assuming we are in McCalib directory
sys.path.append(os.getcwd())
import calibration

def detect_corners(image_files, pattern_size, square_size):
    """
    Detect checkerboard corners in images.
    """
    X_world_list = []
    keypoints_list = []
    visibility_list = []
    image_info_list = []
    
    # Prepare 3D points for the board
    # pattern_size is (cols, rows) of inner corners
    # build_checkerboard_points expects (ph, pw) -> (rows, cols)
    # And it generates points based on that.
    # Note: cv2.findChessboardCorners returns corners row by row (if flags are default)
    
    # calibration.py's build_checkerboard_points:
    # xs = np.arange(pw) * pLength
    # ys = np.arange(ph) * pLength
    # So it generates pw columns and ph rows.
    
    ph = pattern_size[1] # rows
    pw = pattern_size[0] # cols
    
    Xw_board = calibration.build_checkerboard_points(ph, pw, square_size)
    
    print(f"Detecting corners (pattern: {pattern_size})...")
    
    valid_images = 0
    for img_path in sorted(image_files):
        img = cv2.imread(img_path)
        if img is None:
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        
        if found:
            # Refine corners
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
            
            # Reshape to (N, 2)
            uv = corners.reshape(-1, 2)
            
            # Visibility (all found)
            vis = np.ones(uv.shape[0], dtype=bool)
            
            X_world_list.append(Xw_board.copy())
            keypoints_list.append(uv)
            visibility_list.append(vis)
            
            image_info_list.append({
                "file_name": os.path.basename(img_path),
                "width": img.shape[1],
                "height": img.shape[0]
            })
            valid_images += 1
            
    print(f"Found corners in {valid_images}/{len(image_files)} images.")
    return X_world_list, keypoints_list, visibility_list, image_info_list

def run_calibration(X_world_list, keypoints_list, visibility_list, image_info_list, model_type):
    print(f"\n--- Running Calibration: {model_type} ---")
    
    num_images = len(X_world_list)
    img_width = float(image_info_list[0]["width"])
    img_height = float(image_info_list[0]["height"])
    
    # Initial guess
    if model_type == "ds":
        f0 = 0.8 * max(img_width, img_height)
        intrinsics0 = [f0, f0, img_width/2, img_height/2, 0.5, 0.5]
        r0_list = [np.zeros(3) for _ in range(num_images)]
        t0_list = [np.array([0, 0, 1.5]) for _ in range(num_images)] # Guess z=1.5m
    elif model_type == "kb":
        # Use OpenCV init
        fx, fy, cx, cy, k1, k2, k3, k4, rvecs, tvecs = calibration.initialize_kb(
            X_world_list, keypoints_list, image_info_list
        )
        intrinsics0 = [fx, fy, cx, cy, k1, k2, k3, k4]
        r0_list = [r.flatten() for r in rvecs]
        t0_list = [t.flatten() for t in tvecs]
        
    x0 = calibration.pack_params(intrinsics0, r0_list, t0_list)
    
    # Custom bounds for this script to test if relaxation helps
    if model_type == "ds":
        # Relax xi and alpha
        # xi in [-1, 3], alpha in [0, 1]
        fx_lb, fy_lb = 500.0, 500.0
        fx_ub, fy_ub = 4000.0, 4000.0
        cx_lb, cy_lb = 0.0, 0.0
        cx_ub, cy_ub = img_width, img_height
        
        xi_lb, xi_ub = -1.0, 3.0
        alpha_lb, alpha_ub = 0.0, 1.0
        
        intr_lb = np.array([fx_lb, fy_lb, cx_lb, cy_lb, xi_lb, alpha_lb], dtype=np.float64)
        intr_ub = np.array([fx_ub, fy_ub, cx_ub, cy_ub, xi_ub, alpha_ub], dtype=np.float64)
        
        # Extrinsics bounds (same as calibration.py)
        rot_lb = -np.pi * np.ones(3, dtype=np.float64)
        rot_ub = np.pi * np.ones(3, dtype=np.float64)
        tx_lb, ty_lb, tz_lb = -5.0, -5.0, 0.2
        tx_ub, ty_ub, tz_ub = 5.0, 5.0, 20.0
        trans_lb = np.array([tx_lb, ty_lb, tz_lb], dtype=np.float64)
        trans_ub = np.array([tx_ub, ty_ub, tz_ub], dtype=np.float64)
        
        lb_list = [intr_lb]
        ub_list = [intr_ub]
        for _ in range(num_images):
            lb_list.append(rot_lb)
            lb_list.append(trans_lb)
            ub_list.append(rot_ub)
            ub_list.append(trans_ub)
            
        lb = np.concatenate(lb_list, axis=0)
        ub = np.concatenate(ub_list, axis=0)
    else:
        lb, ub = calibration.build_bounds(num_images, img_width, img_height, model_type)
    
    # Optimize
    result = least_squares(
        calibration.calibration_residuals,
        x0,
        args=(X_world_list, keypoints_list, visibility_list, model_type),
        method="trf",
        loss="linear",
        verbose=1,
        max_nfev=100, # Limit iterations for speed
        x_scale="jac",
        bounds=(lb, ub),
    )
    
    # Unpack
    num_intrinsics = 6 if model_type == "ds" else 8
    intrinsics, r_list, t_list = calibration.unpack_params(result.x, num_images, num_intrinsics)
    
    # RMS
    M = result.fun.shape[0]
    rms_px = np.sqrt(2 * result.cost / M)
    print(f"Final RMS: {rms_px:.4f} px")
    
    return intrinsics, r_list, t_list, rms_px

def main():
    # Settings for Scenario 1
    # Based on tests/configs_for_end2end_tests/calib_param_synth_Scenario1.yml
    # number_x_square: 5, number_y_square: 5 -> Inner corners 4x4
    # square_size: 0.192
    
    img_dir = "../data/Blender_Images/Scenario_1/Images/Cam_001"
    pattern_size = (4, 4) # cols, rows
    square_size = 0.192
    
    image_files = glob.glob(os.path.join(img_dir, "*.png"))
    image_files = sorted(image_files)[:20] # Limit to 20 images for speed
    if not image_files:
        print(f"No images found in {img_dir}")
        return

    # 1. Detect Corners
    X_world_list, keypoints_list, visibility_list, image_info_list = detect_corners(
        image_files, pattern_size, square_size
    )
    
    if not X_world_list:
        print("No corners detected. Check pattern size.")
        return

    # 2. Run DS Calibration
    ds_intr, ds_r, ds_t, ds_rms = run_calibration(
        X_world_list, keypoints_list, visibility_list, image_info_list, "ds"
    )
    
    # 3. Run KB Calibration
    kb_intr, kb_r, kb_t, kb_rms = run_calibration(
        X_world_list, keypoints_list, visibility_list, image_info_list, "kb"
    )
    
    # 4. Compare
    print("\n=== Comparison ===")
    print(f"DS RMS: {ds_rms:.4f} px")
    print(f"KB RMS: {kb_rms:.4f} px")
    
    print("\nIntrinsics:")
    print(f"DS: {ds_intr}")
    print(f"KB: {kb_intr}")
    
    # Compare extrinsics for the first image
    print("\nExtrinsics (Image 0):")
    print(f"DS r: {ds_r[0]}")
    print(f"DS t: {ds_t[0]}")
    print(f"KB r: {kb_r[0]}")
    print(f"KB t: {kb_t[0]}")
    
    # Calculate difference in translation
    t_diffs = []
    r_diffs = []
    for i in range(len(ds_t)):
        t_diff = np.linalg.norm(ds_t[i] - kb_t[i])
        t_diffs.append(t_diff)
        
        # Rotation difference (approx)
        r_diff = np.linalg.norm(ds_r[i] - kb_r[i])
        r_diffs.append(r_diff)
        
    print(f"\nMean Translation Difference: {np.mean(t_diffs):.4f} m")
    print(f"Mean Rotation Vector Difference: {np.mean(r_diffs):.4f} rad")
    
    if np.mean(t_diffs) < 0.1: # Threshold?
        print("\nSUCCESS: Extrinsics are consistent between DS and KB.")
    else:
        print("\nWARNING: Extrinsics differ significantly.")

if __name__ == "__main__":
    main()
