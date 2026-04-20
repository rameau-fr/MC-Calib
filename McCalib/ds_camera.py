"""
Double Sphere Camera Model - Clean Implementation
==================================================

A minimal, production-ready implementation of the Double Sphere camera model
for fisheye cameras. Easy to understand, test, and integrate into other projects.

Author: Advanced 3D Vision
License: MIT
"""

import numpy as np
import cv2
import json
from typing import Tuple, Optional


class DoubleSphereCamera:
    """
    Double Sphere Camera Model (Usenko et al., 2018)
    
    A camera model for wide-angle and fisheye lenses with closed-form
    unprojection, making it ideal for real-time applications and PnP.
    
    Parameters
    ----------
    fx, fy : float
        Focal lengths in pixels
    cx, cy : float
        Principal point coordinates
    xi, alpha : float
        Double Sphere distortion parameters
    width, height : int
        Image dimensions in pixels
    
    Examples
    --------
    >>> # Create camera from calibration
    >>> cam = DoubleSphereCamera(
    ...     fx=711.57, fy=711.24, cx=949.18, cy=518.81,
    ...     xi=0.183, alpha=0.809, width=1920, height=1080
    ... )
    >>> 
    >>> # Undistort image
    >>> img_undist, K_new = cam.undistort_image(img)
    >>> 
    >>> # Solve PnP
    >>> success, rvec, tvec = cam.solve_pnp(points_3d, points_2d)
    """
    
    def __init__(self, fx: float, fy: float, cx: float, cy: float,
                 xi: float, alpha: float, width: int, height: int,
                 is_flip: bool = False):
        self.fx = fx
        self.fy = fy
        self.cx_orig = cx  # Store original cx
        self.cy = cy
        self.xi = xi
        self.alpha = alpha
        self.width = width
        self.height = height
        self.is_flip = is_flip
        
        # Adjust cx for flipped images
        # When driver is flipped: cx_flipped = width - cx_original
        self.cx = (width - cx) if is_flip else cx
        
        # Cache for undistortion
        self._mapx = None
        self._mapy = None
        self._K_new = None
    
    @classmethod
    def from_json(cls, json_path: str):
        """Load camera from calibration JSON file."""
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        if 'intrinsics' in data:
            intrinsic = data['intrinsics']
            width = data.get('image_width', 640)
            height = data.get('image_height', 480)
        else:
            cam_data = list(data.values())[0]
            intrinsic = cam_data['intrinsics'][0]['intrinsics']
            resolution = cam_data['resolution'][0]
            width, height = resolution[0], resolution[1]
        
        return cls(
            fx=intrinsic['fx'], fy=intrinsic['fy'],
            cx=intrinsic['cx'], cy=intrinsic['cy'],
            xi=intrinsic['xi'], alpha=intrinsic['alpha'],
            width=width, height=height
        )
    
    # ========================================================================
    # Core Projection/Unprojection
    # ========================================================================
    
    def project(self, points_3d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Project 3D points to 2D pixel coordinates.
        
        Parameters
        ----------
        points_3d : (N, 3) array
            3D points in camera coordinates
            
        Returns
        -------
        points_2d : (N, 2) array
            Pixel coordinates (flipped if is_flip=True)
        valid : (N,) bool array
            Validity mask
        """
        x, y, z = points_3d[..., 0], points_3d[..., 1], points_3d[..., 2]
        
        valid = z > 0
        d1 = np.sqrt(x*x + y*y + z*z)
        z1 = z + self.xi * d1
        d2 = np.sqrt(x*x + y*y + z1*z1)
        den = self.alpha * d2 + (1.0 - self.alpha) * z1
        
        valid &= den > 1e-8
        den = np.maximum(den, 1e-8)
        
        u = self.fx * x / den + self.cx
        v = self.fy * y / den + self.cy
        
        # Flip x-coordinates if driver provides flipped images
        if self.is_flip:
            u = (self.width - 1) - u
        
        return np.stack([u, v], axis=-1), valid
    
    def unproject(self, points_2d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Unproject 2D pixels to 3D unit rays (closed-form).
        
        Parameters
        ----------
        points_2d : (N, 2) array
            Pixel coordinates (flipped if is_flip=True)
            
        Returns
        -------
        rays : (N, 3) array
            Unit 3D direction vectors
        valid : (N,) bool array
            Validity mask
        """
        u, v = points_2d[..., 0], points_2d[..., 1]
        
        # Flip x-coordinates if driver provides flipped images
        if self.is_flip:
            u = (self.width - 1) - u
        
        mx = (u - self.cx) / self.fx
        my = (v - self.cy) / self.fy
        r2 = mx*mx + my*my
        
        # Validity check
        s = 1.0 - (2.0 * self.alpha - 1.0) * r2
        valid = s >= 0
        s = np.maximum(s, 0.0)
        
        # Closed-form unprojection
        mz = (1.0 - self.alpha*self.alpha * r2) / (self.alpha * np.sqrt(s) + (1.0 - self.alpha))
        k = (mz * self.xi + np.sqrt(mz*mz + (1.0 - self.xi*self.xi) * r2)) / np.maximum(mz*mz + r2, 1e-10)
        
        ray = np.stack([k * mx, k * my, k * mz - self.xi], axis=-1)
        norm = np.linalg.norm(ray, axis=-1, keepdims=True)
        ray = ray / np.maximum(norm, 1e-10)
        
        return ray, valid
    
    # ========================================================================
    # Image Undistortion
    # ========================================================================
    
    def compute_K_new(self, balance: float = 0.5) -> np.ndarray:
        """
        Compute optimal K matrix for undistorted image.
        
        Parameters
        ----------
        balance : float
            0.0 = more FOV (40% of original focal length)
            0.5 = balanced (60% of original) - default
            1.0 = less FOV (80% of original)
            
        Returns
        -------
        K_new : (3, 3) array
            New intrinsic matrix
        """
        focal_scale = 0.4 + balance * 0.4  # Range: 0.4 to 0.8
        f_new = ((self.fx + self.fy) / 2.0) * focal_scale
        
        return np.array([
            [f_new, 0, self.width / 2.0],
            [0, f_new, self.height / 2.0],
            [0, 0, 1]
        ], dtype=np.float64)
    
    def get_undistortion_maps(self, K_new: Optional[np.ndarray] = None
                             ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate undistortion maps for cv2.remap.
        
        Parameters
        ----------
        K_new : (3, 3) array, optional
            New intrinsic matrix. If None, computed automatically.
            
        Returns
        -------
        mapx, mapy : (H, W) arrays
            Maps for cv2.remap
        K_new : (3, 3) array
            Intrinsic matrix used
        """
        if self._mapx is not None and K_new is None:
            return self._mapx, self._mapy, self._K_new
        
        if K_new is None:
            K_new = self.compute_K_new()
        
        fx_new, fy_new = K_new[0, 0], K_new[1, 1]
        cx_new, cy_new = K_new[0, 2], K_new[1, 2]
        
        # Create undistorted pixel grid
        x = np.arange(self.width, dtype=np.float32)
        y = np.arange(self.height, dtype=np.float32)
        x_grid, y_grid = np.meshgrid(x, y, indexing='xy')
        
        # Convert to normalized coordinates and create rays
        mx = (x_grid - cx_new) / fx_new
        my = (y_grid - cy_new) / fy_new
        rays = np.stack([mx, my, np.ones_like(mx)], axis=-1)
        rays = rays / np.linalg.norm(rays, axis=-1, keepdims=True)
        
        # Project back to distorted image
        distorted_pts, valid = self.project(rays)
        
        mapx = distorted_pts[..., 0].astype(np.float32)
        mapy = distorted_pts[..., 1].astype(np.float32)
        mapx[~valid] = -1
        mapy[~valid] = -1
        
        if K_new is None or (K_new == self.compute_K_new()).all():
            self._mapx, self._mapy, self._K_new = mapx, mapy, K_new
        
        return mapx, mapy, K_new
    
    def undistort_image(self, img: np.ndarray, K_new: Optional[np.ndarray] = None
                       ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Undistort fisheye image to pinhole.
        
        Parameters
        ----------
        img : (H, W, C) or (H, W) array
            Input distorted image
        K_new : (3, 3) array, optional
            New intrinsic matrix
            
        Returns
        -------
        img_undist : array
            Undistorted image
        K_new : (3, 3) array
            Intrinsic matrix for undistorted image
        """
        mapx, mapy, K_new = self.get_undistortion_maps(K_new)
        img_undist = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return img_undist, K_new
    
    # ========================================================================
    # Keypoint Transformation
    # ========================================================================
    
    def undistort_points(self, points_dist: np.ndarray, K_new: np.ndarray
                        ) -> Tuple[np.ndarray, np.ndarray]:
        """Transform keypoints from distorted to undistorted space."""
        rays, valid = self.unproject(points_dist)
        rays_norm = rays / (rays[:, 2:3] + 1e-10)
        
        u = K_new[0, 0] * rays_norm[:, 0] + K_new[0, 2]
        v = K_new[1, 1] * rays_norm[:, 1] + K_new[1, 2]
        
        return np.column_stack([u, v]), valid
    
    def distort_points(self, points_undist: np.ndarray, K_new: np.ndarray
                      ) -> Tuple[np.ndarray, np.ndarray]:
        """Transform keypoints from undistorted to distorted space."""
        mx = (points_undist[:, 0] - K_new[0, 2]) / K_new[0, 0]
        my = (points_undist[:, 1] - K_new[1, 2]) / K_new[1, 1]
        rays = np.column_stack([mx, my, np.ones(len(mx))])
        rays = rays / np.linalg.norm(rays, axis=1, keepdims=True)
        
        return self.project(rays)
    
    # ========================================================================
    # PnP Pose Estimation
    # ========================================================================
    
    def solve_pnp(self, points_3d: np.ndarray, points_2d: np.ndarray,
                  method: int = cv2.SOLVEPNP_ITERATIVE
                 ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Solve PnP for fisheye camera.
        
        This method handles fisheye distortion by unprojecting to rays
        and solving PnP in normalized space.
        
        Parameters
        ----------
        points_3d : (N, 3) array
            3D points in world coordinates
        points_2d : (N, 2) array
            2D keypoints in distorted image
        method : int
            OpenCV PnP method (e.g., cv2.SOLVEPNP_ITERATIVE)
            
        Returns
        -------
        success : bool
        rvec : (3,) array or None
            Rotation vector
        tvec : (3,) array or None
            Translation vector
        """
        rays, valid = self.unproject(points_2d)
        
        if not valid.all():
            points_3d = points_3d[valid]
            rays = rays[valid]
            if len(points_3d) < 4:
                return False, None, None
        
        rays_norm = rays / (rays[:, 2:3] + 1e-10)
        points_2d_norm = rays_norm[:, :2]
        
        success, rvec, tvec = cv2.solvePnP(
            points_3d.astype(np.float64),
            points_2d_norm.astype(np.float64),
            np.eye(3, dtype=np.float64),
            np.zeros(5, dtype=np.float64),
            flags=method
        )
        
        if success:
            rvec = rvec.squeeze()
            tvec = tvec.squeeze()
        
        return success, rvec, tvec
    
    # ========================================================================
    # Visualization
    # ========================================================================
    
    def draw_axes(self, img: np.ndarray, rvec: np.ndarray, tvec: np.ndarray,
                  axis_length: float = 0.1, K: Optional[np.ndarray] = None
                 ) -> np.ndarray:
        """
        Draw 3D coordinate axes on image.
        
        Parameters
        ----------
        img : array
            Image to draw on
        rvec, tvec : (3,) arrays
            Pose (rotation and translation vectors)
        axis_length : float
            Length of axes in meters
        K : (3, 3) array, optional
            If None, draws on distorted image. Otherwise, draws on undistorted.
            
        Returns
        -------
        img_out : array
            Image with drawn axes
        """
        img_out = img.copy()
        
        # Define axes in 3D
        axes_3d = np.array([
            [0, 0, 0],
            [axis_length, 0, 0],  # X: Red
            [0, axis_length, 0],  # Y: Green
            [0, 0, axis_length]   # Z: Blue
        ])
        
        # Transform to camera coordinates
        R, _ = cv2.Rodrigues(rvec)
        axes_cam = (R @ axes_3d.T).T + tvec
        
        # Project
        if K is None:
            # Distorted image
            axes_2d, valid = self.project(axes_cam)
        else:
            # Undistorted image
            axes_2d_hom = (K @ axes_cam.T).T
            axes_2d = axes_2d_hom[:, :2] / axes_2d_hom[:, 2:3]
            valid = axes_cam[:, 2] > 0
        
        if not valid.all():
            return img_out
        
        axes_2d_int = axes_2d.astype(np.int32)
        origin = tuple(axes_2d_int[0])
        
        # Draw axes
        cv2.arrowedLine(img_out, origin, tuple(axes_2d_int[1]), (0, 0, 255), 3, tipLength=0.3)  # X: Red
        cv2.arrowedLine(img_out, origin, tuple(axes_2d_int[2]), (0, 255, 0), 3, tipLength=0.3)  # Y: Green
        cv2.arrowedLine(img_out, origin, tuple(axes_2d_int[3]), (255, 0, 0), 3, tipLength=0.3)  # Z: Blue
        cv2.circle(img_out, origin, 5, (255, 255, 255), -1)
        
        return img_out


# ============================================================================
# Convenience Functions
# ============================================================================

def undistort_fisheye(img: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                      xi: float, alpha: float) -> Tuple[np.ndarray, np.ndarray]:
    """Quick function to undistort a fisheye image."""
    h, w = img.shape[:2]
    cam = DoubleSphereCamera(fx, fy, cx, cy, xi, alpha, w, h)
    return cam.undistort_image(img)


def solve_pnp_fisheye(points_3d: np.ndarray, points_2d: np.ndarray,
                      fx: float, fy: float, cx: float, cy: float,
                      xi: float, alpha: float
                     ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
    """Quick function to solve PnP for fisheye camera."""
    cam = DoubleSphereCamera(fx, fy, cx, cy, xi, alpha, 640, 480)
    return cam.solve_pnp(points_3d, points_2d)
