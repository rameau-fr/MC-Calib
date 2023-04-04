#include <chrono>
#include <iomanip>
#include <stdio.h>

#include "McCalib.hpp"
#include "logger.h"

void runCalibrationWorkflow(std::string config_path) {
  // Instantiate the calibration and initialize the parameters
  McCalib::Calibration Calib(config_path);
  Calib.boardExtraction();
  LOG_INFO << "Board extraction done!";

  // Intrinsic calibration of the cameras
  LOG_INFO << "Intrinsic calibration initiated";
  Calib.initIntrinsic();
  LOG_INFO << "Intrinsic Calibration done!";

  // Calibrate 3D Objects
  LOG_INFO << "3D Object calibration initiated";
  Calib.calibrate3DObjects();
  LOG_INFO << "3D Object calibration done!";

  // Calibrate camera groups
  LOG_INFO << "Camera group calibration initiated";
  Calib.calibrateCameraGroup();
  LOG_INFO << "Camera group calibration done!";

  // Merge objects again to deal with boards visible simultaneously from camera
  // groups
  Calib.merge3DObjects();

  // Calibrate Non-Overlapping cameras
  LOG_INFO << "Non-overlapping calibration initiated";
  Calib.findPairObjectForNonOverlap();
  Calib.findPoseNoOverlapAllCamGroup();
  LOG_INFO << "Non-overlapping calibration done!";

  // merge camera groups 1
  LOG_INFO << "Merge cameras and objets initiated";
  Calib.initInterCamGroupGraph();
  Calib.mergeCameraGroup();
  Calib.mergeAllCameraGroupObs();
  // Merge objects
  Calib.merge3DObjects();
  // merge camera groups 2
  Calib.initInterCamGroupGraph();
  Calib.mergeCameraGroup();
  Calib.mergeAllCameraGroupObs();
  Calib.estimatePoseAllObjects();
  Calib.computeAllObjPoseInCameraGroup();
  LOG_INFO << "Merge cameras and objets done!";

  // Final Optimization
  LOG_INFO << "Final refinement initiated";
  // Calib.reproErrorAllCamGroup(); // this is just to check the reprojection
  // error before optimization Calib.refineAllCameraGroup(); // Refine Camera
  // only
  Calib.refineAllCameraGroupAndObjects();
  // Optimize everything including intrinsics
  if (Calib.fix_intrinsic_ == 0) {
    Calib.refineAllCameraGroupAndObjectsAndIntrinsic();
  }

  Calib.reproErrorAllCamGroup();
  LOG_INFO << "Final refinement done";

  // Save images reprojection
  if (Calib.save_detect_ == 1)
    Calib.saveDetectionAllCam();
  if (Calib.save_repro_ == 1)
    Calib.saveReprojectionAllCam();

  // Save camera parameters
  LOG_INFO << "Save parameters";
  Calib.saveCamerasParams();
  Calib.save3DObj();
  Calib.save3DObjPose();
  Calib.saveReprojectionErrorToFile();
  LOG_INFO << "mean reprojection error :: "
           << Calib.computeAvgReprojectionError() << std::endl;
}

int main(int argc, char *argv[]) {
  (void)argc; // casting to fix -Werror=unused-parameter
  const std::string config_path = argv[1];
  const bool is_file_available =
      boost::filesystem::exists(config_path) && config_path.length() > 0;
  if (!is_file_available) {
    LOG_FATAL << "Config path '" << config_path << "' doesn't exist.";
    return -1;
  }

  auto start = std::chrono::high_resolution_clock::now();
  runCalibrationWorkflow(config_path);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::seconds>(stop - start);
  LOG_INFO << "Calibration took " << duration.count() << " seconds";

  return 0;
}