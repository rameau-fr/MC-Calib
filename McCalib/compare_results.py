import cv2
import numpy as np
import os
import yaml

def load_calibration(file_path):
    with open(file_path, 'r') as f:
        # Skip the first line if it's %YAML:1.0
        lines = f.readlines()
        if lines[0].startswith('%YAML'):
            content = ''.join(lines[1:])
        else:
            content = ''.join(lines)
            
    # OpenCV YAML parsing is tricky with standard yaml lib.
    # Let's use cv2.FileStorage
    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    
    cameras = {}
    nb_camera = int(fs.getNode("nb_camera").real())
    
    for i in range(nb_camera):
        cam_node = fs.getNode(f"Camera_{i}")
        intrinsics = cam_node.getNode("Intrinsics").mat()
        distortion = cam_node.getNode("Distortion").mat()
        
        # Extrinsics might be stored as 'CameraPose' or similar?
        # The file format needs to be checked.
        # Based on McCalib.cpp saveCamerasParams:
        # "Camera_i" -> "Intrinsics", "Distortion", "ImageSize", "ReprojectionError"
        # Wait, where are extrinsics?
        # They are likely in "CameraGroup_0" -> "Camera_i_Pose" ?
        # Or "Camera_i" -> "Pose" ?
        pass

    # Let's check the file content first to be sure.
    return fs

def compare_extrinsics(kb_path, ds_path):
    print(f"Loading KB: {kb_path}")
    fs_kb = cv2.FileStorage(kb_path, cv2.FILE_STORAGE_READ)
    print(f"Loading DS: {ds_path}")
    fs_ds = cv2.FileStorage(ds_path, cv2.FILE_STORAGE_READ)
    
    nb_camera = int(fs_kb.getNode("nb_camera").real())
    print(f"Number of cameras: {nb_camera}")
    
    for i in range(nb_camera):
        # Structure is camera_{i} -> camera_pose_matrix
        cam_node_kb = fs_kb.getNode(f"camera_{i}")
        cam_node_ds = fs_ds.getNode(f"camera_{i}")
        
        if cam_node_kb.empty() or cam_node_ds.empty():
            print(f"Camera {i} not found in one of the files.")
            continue
            
        pose_kb = cam_node_kb.getNode("camera_pose_matrix").mat()
        pose_ds = cam_node_ds.getNode("camera_pose_matrix").mat()
        
        print(f"--- Camera {i} ---")
        # print("KB Pose:\n", pose_kb)
        # print("DS Pose:\n", pose_ds)
        
        if pose_kb is not None and pose_ds is not None:
            diff = np.abs(pose_kb - pose_ds)
            # print("Difference:\n", diff)
            print("Max Element Diff:", np.max(diff))
            
            # Decompose to R and T
            R_kb = pose_kb[:3, :3]
            T_kb = pose_kb[:3, 3]
            R_ds = pose_ds[:3, :3]
            T_ds = pose_ds[:3, 3]
            
            trans_diff = np.linalg.norm(T_kb - T_ds)
            
            # Rotation difference (angle)
            R_diff = R_kb @ R_ds.T
            trace = np.trace(R_diff)
            # Clip to avoid numerical errors
            val = (trace - 1) / 2
            val = np.clip(val, -1, 1)
            angle = np.arccos(val)
            
            print(f"Translation Diff Norm: {trans_diff:.6f}")
            print(f"Rotation Diff Angle (rad): {angle:.6f}")

if __name__ == "__main__":
    kb_res = "../data/Blender_Images/Scenario_1/Results_KB/calibrated_cameras_data.yml"
    ds_res = "../data/Blender_Images/Scenario_1/Results_DS/calibrated_cameras_data.yml"
    
    # Adjust paths to absolute
    base_path = "/home/haruka_nanase/Desktop/Adv3D/My_Portfolio/MC-Calib"
    kb_res = os.path.join(base_path, "data/Blender_Images/Scenario_1/Results_KB/calibrated_cameras_data.yml")
    ds_res = os.path.join(base_path, "data/Blender_Images/Scenario_1/Results_DS/calibrated_cameras_data.yml")
    
    if not os.path.exists(kb_res):
        print(f"KB results not found at {kb_res}")
    if not os.path.exists(ds_res):
        print(f"DS results not found at {ds_res}")
        
    if os.path.exists(kb_res) and os.path.exists(ds_res):
        compare_extrinsics(kb_res, ds_res)
