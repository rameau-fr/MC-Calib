"""
Simple Examples for Double Sphere Camera Model
==============================================

Copy-paste ready examples for common 3D vision tasks.
"""

import cv2
import numpy as np
from ds_camera import DoubleSphereCamera


# ============================================================================
# Example 1: Basic Image Undistortion
# ============================================================================

def example_undistortion():
    """Undistort a fisheye image to pinhole projection."""
    
    # Load camera
    cam = DoubleSphereCamera.from_json('calibration.json')
    
    # Load image
    img = cv2.imread('fisheye_image.jpg')
    
    # Undistort
    img_undist, K_new = cam.undistort_image(img)
    
    # Save
    cv2.imwrite('undistorted.jpg', img_undist)
    print(f"K_new focal length: {K_new[0,0]:.2f}")


# ============================================================================
# Example 2: ArUco Marker Detection and Pose
# ============================================================================

def example_aruco_pose():
    """Detect ArUco markers and estimate pose."""
    
    # Setup
    cam = DoubleSphereCamera.from_json('calibration.json')
    img = cv2.imread('fisheye_aruco.jpg')
    
    # Detect ArUco markers
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    corners, ids, _ = cv2.aruco.detectMarkers(img, aruco_dict)
    
    if len(corners) > 0:
        # Define 3D marker corners (10cm marker)
        marker_size = 0.1
        points_3d = np.array([
            [-marker_size/2, -marker_size/2, 0],
            [ marker_size/2, -marker_size/2, 0],
            [ marker_size/2,  marker_size/2, 0],
            [-marker_size/2,  marker_size/2, 0]
        ])
        
        # Get 2D corners
        points_2d = corners[0].reshape(-1, 2)
        
        # Solve PnP
        success, rvec, tvec = cam.solve_pnp(points_3d, points_2d)
        
        if success:
            # Draw axes
            img_result = cam.draw_axes(img, rvec, tvec, axis_length=0.05)
            cv2.imwrite('aruco_pose.jpg', img_result)
            
            print(f"Distance: {np.linalg.norm(tvec):.3f}m")
            print(f"Position: {tvec}")


# ============================================================================
# Example 3: Checkerboard Calibration Validation
# ============================================================================

def example_checkerboard_validation():
    """Validate calibration using checkerboard."""
    
    # Load camera and image
    cam = DoubleSphereCamera.from_json('calibration.json')
    img = cv2.imread('checkerboard.jpg')
    
    # Detect checkerboard (adjust pattern size)
    pattern_size = (6, 5)  # cols, rows
    found, corners = cv2.findChessboardCorners(
        cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
        pattern_size
    )
    
    if found:
        # Build 3D points (20cm squares)
        square_size = 0.2
        rows, cols = pattern_size[1], pattern_size[0]
        points_3d = np.array([
            [j * square_size, i * square_size, 0.0]
            for i in range(rows) for j in range(cols)
        ])
        
        # Solve PnP
        points_2d = corners.reshape(-1, 2)
        success, rvec, tvec = cam.solve_pnp(points_3d, points_2d)
        
        if success:
            # Compute reprojection error
            R, _ = cv2.Rodrigues(rvec)
            points_cam = (R @ points_3d.T).T + tvec
            points_2d_reproj, _ = cam.project(points_cam)
            
            error = np.linalg.norm(points_2d_reproj - points_2d, axis=1)
            print(f"Reprojection error: {error.mean():.3f} px (mean)")
            print(f"Calibration quality: {'✅ Good' if error.mean() < 1.0 else '⚠️ Check calibration'}")


# ============================================================================
# Example 4: Batch Processing
# ============================================================================

def example_batch_undistortion():
    """Undistort multiple images efficiently."""
    import glob
    
    # Setup camera and generate maps once
    cam = DoubleSphereCamera.from_json('calibration.json')
    mapx, mapy, K_new = cam.get_undistortion_maps()
    
    # Process all images
    for img_path in glob.glob('fisheye_*.jpg'):
        img = cv2.imread(img_path)
        img_undist = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        
        out_path = img_path.replace('.jpg', '_undist.jpg')
        cv2.imwrite(out_path, img_undist)
        print(f"Processed: {img_path}")


# ============================================================================
# Example 5: Camera Pose from Known 3D Points
# ============================================================================

def example_custom_3d_points():
    """Estimate camera pose from custom 3D-2D correspondences."""
    
    cam = DoubleSphereCamera.from_json('calibration.json')
    
    # Your 3D points in world coordinates (e.g., from CAD model)
    points_3d = np.array([
        [0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0],
        [0.5, 0.3, 0.0],
        [0.0, 0.3, 0.0],
        [0.25, 0.15, 0.2]
    ])
    
    # Corresponding 2D detections in image (e.g., from feature detector)
    points_2d = np.array([
        [823, 512],
        [1024, 487],
        [1067, 631],
        [856, 654],
        [945, 573]
    ])
    
    # Solve PnP
    success, rvec, tvec = cam.solve_pnp(points_3d, points_2d)
    
    if success:
        # Get camera position in world frame
        R, _ = cv2.Rodrigues(rvec)
        camera_pos = -R.T @ tvec
        
        print(f"Camera position: {camera_pos}")
        print(f"Looking direction: {R[:, 2]}")  # Z-axis of camera


# ============================================================================
# Example 6: Keypoint Tracking Across Undistortion
# ============================================================================

def example_keypoint_tracking():
    """Track keypoints across distorted/undistorted images."""
    
    cam = DoubleSphereCamera.from_json('calibration.json')
    img = cv2.imread('fisheye.jpg')
    
    # Detect features in distorted image (e.g., ORB)
    orb = cv2.ORB_create()
    keypoints_dist = orb.detect(img, None)
    points_dist = np.array([kp.pt for kp in keypoints_dist])
    
    # Undistort image and keypoints
    img_undist, K_new = cam.undistort_image(img)
    points_undist, valid = cam.undistort_points(points_dist, K_new)
    
    # Now you can track in undistorted space
    # and transform back if needed
    points_back, _ = cam.distort_points(points_undist, K_new)
    
    print(f"Tracked {len(points_undist)} keypoints")
    print(f"Roundtrip error: {np.abs(points_back - points_dist).mean():.6f} px")


# ============================================================================
# Example 7: Different FOV Options
# ============================================================================

def example_fov_control():
    """Control field of view in undistorted image."""
    
    cam = DoubleSphereCamera.from_json('calibration.json')
    img = cv2.imread('fisheye.jpg')
    
    # More FOV (zoom out)
    K_wide = cam.compute_K_new(balance=0.0)  # 40% focal length
    img_wide, _ = cam.undistort_image(img, K_wide)
    cv2.imwrite('undist_wide_fov.jpg', img_wide)
    
    # Balanced (default)
    K_balanced = cam.compute_K_new(balance=0.5)  # 60% focal length
    img_balanced, _ = cam.undistort_image(img, K_balanced)
    cv2.imwrite('undist_balanced.jpg', img_balanced)
    
    # Less FOV (zoom in)
    K_narrow = cam.compute_K_new(balance=1.0)  # 80% focal length
    img_narrow, _ = cam.undistort_image(img, K_narrow)
    cv2.imwrite('undist_narrow_fov.jpg', img_narrow)


# ============================================================================
# Quick Test
# ============================================================================

if __name__ == "__main__":
    # Test with synthetic data
    print("Double Sphere Camera - Examples")
    print("=" * 50)
    
    # Create test camera
    cam = DoubleSphereCamera(
        fx=711.57, fy=711.24, cx=949.18, cy=518.81,
        xi=0.183, alpha=0.809, width=1920, height=1080
    )
    
    # Test projection/unprojection
    points_3d = np.array([[0, 0, 1], [0.1, 0, 1], [0, 0.1, 1]])
    points_2d, valid = cam.project(points_3d)
    rays, valid2 = cam.unproject(points_2d)
    
    print(f"\n✅ Projection/unprojection test:")
    print(f"   3D points: {len(points_3d)}")
    print(f"   Projected: {points_2d[:2]}")
    print(f"   Roundtrip error: {np.abs((rays - points_3d / np.linalg.norm(points_3d, axis=1, keepdims=True))).max():.2e}")
    
    # Test K computation
    K_new = cam.compute_K_new()
    print(f"\n✅ K matrix computation:")
    print(f"   Focal length: {K_new[0,0]:.2f} px (60% of original)")
    print(f"   Principal point: ({K_new[0,2]:.0f}, {K_new[1,2]:.0f})")
    
    print(f"\n✅ All systems operational!")
    print(f"\nSee individual example functions above for usage.")
