#pragma once

#include <opencv2/core/version.hpp>

// OpenCV 4.x ships ChArUco in opencv2/aruco/charuco.hpp.
// OpenCV 5.x may expose it via objdetect headers.
#if __has_include(<opencv2/aruco/charuco.hpp>)
#include <opencv2/aruco/charuco.hpp>
#define MC_CALIB_HAS_LEGACY_CHARUCO_INTERPOLATE 1
#elif __has_include(<opencv2/objdetect/charuco_detector.hpp>)
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#define MC_CALIB_HAS_LEGACY_CHARUCO_INTERPOLATE 0
#elif __has_include(<opencv2/objdetect.hpp>)
#include <opencv2/objdetect.hpp>
#define MC_CALIB_HAS_LEGACY_CHARUCO_INTERPOLATE 0
#else
#error                                                                         \
    "Could not find OpenCV ArUco/ChArUco headers. Install OpenCV with contrib modules."
#endif

// Helper macro for version-based OpenCV API switches.
#define MC_CALIB_OPENCV_AT_LEAST(major, minor)                                 \
  ((CV_VERSION_MAJOR > (major)) ||                                             \
   (CV_VERSION_MAJOR == (major) && CV_VERSION_MINOR >= (minor)))

// ArUco API changed in OpenCV 4.7.
#define MC_CALIB_USE_LEGACY_ARUCO_API (!MC_CALIB_OPENCV_AT_LEAST(4, 7))