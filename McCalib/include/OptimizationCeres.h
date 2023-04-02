#ifndef OPTIMIZATIONCERES_H
#define OPTIMIZATIONCERES_H

#endif // OPTIMIZATIONCERES_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <eigen3/Eigen/Dense>

// Intrinsic and board pose refinement
struct ReprojectionError {
  ReprojectionError(double u, double v, double x, double y, double z,
                    int distortion_type)
      : u(u), v(v), x(x), y(y), z(z), distortion_type(distortion_type) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const Intrinsics,
                  T *residuals) const {

    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    const T point[3] = {T(x), T(y), T(z)};
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Normalization on the camera plane
    p[0] /= p[2];
    p[1] /= p[2];

    if (distortion_type == 0) // perspective brown
    {
      // apply distorsion
      const T k1 = Intrinsics[4]; // radial 1
      const T k2 = Intrinsics[5]; // radial 2
      const T k3 = Intrinsics[8]; // radial 3
      const T p1 = Intrinsics[6]; // tangential 1
      const T p2 = Intrinsics[7]; // tangential 2
      T r2 = p[0] * p[0] + p[1] * p[1];
      T r4 = r2 * r2;
      T r6 = r4 * r2;
      T r_coeff = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);
      T xd = p[0] * r_coeff + T(2) * p1 * p[0] * p[1] +
             p2 * (r2 + T(2) * p[0] * p[0]);
      T yd = p[1] * r_coeff + p1 * (r2 + T(2) * (p[1] * p[1])) +
             T(2) * p2 * p[0] * p[1];

      // Project on the image plane
      T up = Intrinsics[0] * xd +
             Intrinsics[2]; // Intrinsic[0] = fx, Intrinsic[1]=fy,
                            // Intrinsic[2]=u0...
      T vp = Intrinsics[1] * yd + Intrinsics[3];

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }

    if (distortion_type == 1) // fisheye
    {
      // apply distorsion
      // (source : https://www.programmersought.com/article/72251092167/)
      const T k1 = Intrinsics[4]; //
      const T k2 = Intrinsics[5]; //
      const T k3 = Intrinsics[6]; //
      const T k4 = Intrinsics[7]; //
      T r2 = p[0] * p[0] + p[1] * p[1];
      using std::atan;
      using std::sqrt;
      T r = sqrt(r2);
      // auto r=sqrt<T>(r2);
      T theta = atan(r);
      // T theta = atan<T>(r);
      T theta2 = theta * theta, theta3 = theta2 * theta,
        theta4 = theta2 * theta2, theta5 = theta4 * theta;
      T theta6 = theta3 * theta3, theta7 = theta6 * theta,
        theta8 = theta4 * theta4, theta9 = theta8 * theta;
      T theta_d = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
      T inv_r = r > T(1e-8) ? T(1.0) / r : T(1);
      T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);
      //    T inv_r =  T(1)/r;
      //    T cdist = theta_d * inv_r
      T xd = p[0] * cdist;
      T yd = p[1] * cdist;

      // Project on the image plane
      T up = Intrinsics[0] * xd +
             Intrinsics[2]; // Intrinsic[0] = fx, Intrinsic[1]=fy,
                            // Intrinsic[2]=u0...
      T vp = Intrinsics[1] * yd + Intrinsics[3];

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }

    return true;
  }

  static ceres::CostFunction *Create(const double u, const double v,
                                     const double x, const double y,
                                     const double z,
                                     const int distortion_type) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 9>(
        new ReprojectionError(u, v, x, y, z, distortion_type)));
  }
  double u, v;
  double x;
  double y;
  double z;
  int distortion_type;
};

// 3D object refinement (board pose + object pose)
struct ReprojectionError_3DObjRef {
  ReprojectionError_3DObjRef(double u, double v, double x, double y, double z,
                             double focal_x, double focal_y, double u0,
                             double v0, double k1, double k2, double k3,
                             double p1, double p2, bool refine_board,
                             int distortion_type)
      : u(u), v(v), x(x), y(y), z(z), focal_x(focal_x), focal_y(focal_y),
        u0(u0), v0(v0), k1(k1), k2(k2), k3(k3), p1(p1), p2(p2),
        refine_board(refine_board), distortion_type(distortion_type) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const boardtrans,
                  T *residuals) const {

    // apply transformation to the board
    T pboard[3];
    const T point[3] = {T(x), T(y), T(z)};
    // refine_board == 0 then the board is the reference and should not be
    // refined
    if (refine_board != 0) {
      ceres::AngleAxisRotatePoint(boardtrans, point, pboard);
      pboard[0] += boardtrans[3];
      pboard[1] += boardtrans[4];
      pboard[2] += boardtrans[5];
    } else {
      pboard[0] = point[0];
      pboard[1] = point[1];
      pboard[2] = point[2];
    }

    // camera[0,1,2] are the angle-axis rotation for the camera.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, pboard, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Normalization on the camera plane
    p[0] /= p[2];
    p[1] /= p[2];

    if (distortion_type == 0) // perspective brown
    {
      // apply distorsion
      T r2 = p[0] * p[0] + p[1] * p[1];
      T r4 = r2 * r2;
      T r6 = r4 * r2;
      T r_coeff = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);
      T xd = p[0] * r_coeff + T(2) * p1 * p[0] * p[1] +
             p2 * (r2 + T(2) * p[0] * p[0]);
      T yd = p[1] * r_coeff + p1 * (r2 + T(2) * (p[1] * p[1])) +
             T(2) * p2 * p[0] * p[1];

      // Project on the image plane
      T up = T(focal_x) * xd + T(u0);
      T vp = T(focal_y) * yd + T(v0);

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }

    if (distortion_type == 1) // fisheye
    {
      // apply distorsion
      // (source : https://www.programmersought.com/article/72251092167/)

      T r2 = p[0] * p[0] + p[1] * p[1];
      using std::atan;
      using std::sqrt;
      T r = sqrt(r2);
      // auto r=sqrt<T>(r2);
      T theta = atan(r);
      // T theta = atan<T>(r);
      T theta2 = theta * theta, theta3 = theta2 * theta,
        theta4 = theta2 * theta2, theta5 = theta4 * theta;
      T theta6 = theta3 * theta3, theta7 = theta6 * theta,
        theta8 = theta4 * theta4, theta9 = theta8 * theta;
      T theta_d = theta + k1 * theta3 + k2 * theta5 + p1 * theta7 + p2 * theta9;
      T inv_r = r > T(1e-8) ? T(1.0) / r : T(1);
      T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);
      //    T inv_r =  T(1)/r;
      //    T cdist = theta_d * inv_r
      T xd = p[0] * cdist;
      T yd = p[1] * cdist;

      // Project on the image plane
      T up = T(focal_x) * xd + T(u0);
      T vp = T(focal_y) * yd + T(v0);

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *
  Create(const double u, const double v, const double x, const double y,
         const double z, const double focal_x, const double focal_y,
         const double u0, const double v0, const double k1, const double k2,
         const double k3, const double p1, const double p2,
         const bool refine_board, const int distortion_type) {
    return (
        new ceres::AutoDiffCostFunction<ReprojectionError_3DObjRef, 2, 6, 6>(
            new ReprojectionError_3DObjRef(u, v, x, y, z, focal_x, focal_y, u0,
                                           v0, k1, k2, k3, p1, p2, refine_board,
                                           distortion_type)));
  }

  double u, v;
  double x;
  double y;
  double z;
  double focal_x;
  double focal_y;
  double u0;
  double v0;
  double k1;
  double k2;
  double k3;
  double p1;
  double p2;
  bool refine_board;
  int distortion_type;
};

// Refine camera group (3D object pose and relative camera pose)
// 3D object refinement (board pose + object pose)
struct ReprojectionError_CameraGroupRef {
  ReprojectionError_CameraGroupRef(double u, double v, double x, double y,
                                   double z, double focal_x, double focal_y,
                                   double u0, double v0, double k1, double k2,
                                   double k3, double p1, double p2,
                                   bool refine_camera, int distortion_type)
      : u(u), v(v), x(x), y(y), z(z), focal_x(focal_x), focal_y(focal_y),
        u0(u0), v0(v0), k1(k1), k2(k2), k3(k3), p1(p1), p2(p2),
        refine_camera(refine_camera), distortion_type(distortion_type) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const object_pose,
                  T *residuals) const {

    // 1. apply transformation to the object (to expressed in the current
    // camera)
    T pobj[3];
    const T point[3] = {T(x), T(y), T(z)};
    ceres::AngleAxisRotatePoint(object_pose, point, pobj);
    pobj[0] += object_pose[3];
    pobj[1] += object_pose[4];
    pobj[2] += object_pose[5];

    // 2. Refine the camera if it is not the referential
    if (refine_camera != 0) {
      T pobj_refine[3];
      ceres::AngleAxisRotatePoint(camera, pobj, pobj_refine);
      std::copy_n(pobj_refine, 3, pobj);
      pobj[0] += camera[3];
      pobj[1] += camera[4];
      pobj[2] += camera[5];
    }

    // Normalization on the camera plane
    pobj[0] /= pobj[2];
    pobj[1] /= pobj[2];

    if (distortion_type == 0) // perspective brown
    {
      // apply distorsion
      T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
      T r4 = r2 * r2;
      T r6 = r4 * r2;
      T r_coeff = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);
      T xd = pobj[0] * r_coeff + T(2) * p1 * pobj[0] * pobj[1] +
             p2 * (r2 + T(2) * pobj[0] * pobj[0]);
      T yd = pobj[1] * r_coeff + p1 * (r2 + T(2) * (pobj[1] * pobj[1])) +
             T(2) * p2 * pobj[0] * pobj[1];

      // Project on the image plane
      T up = T(focal_x) * xd + T(u0);
      T vp = T(focal_y) * yd + T(v0);

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }

    if (distortion_type == 1) // fisheye
    {
      // apply distorsion
      // (source : https://www.programmersought.com/article/72251092167/)

      T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
      using std::atan;
      using std::sqrt;
      T r = sqrt(r2);
      // auto r=sqrt<T>(r2);
      T theta = atan(r);
      // T theta = atan<T>(r);
      T theta2 = theta * theta, theta3 = theta2 * theta,
        theta4 = theta2 * theta2, theta5 = theta4 * theta;
      T theta6 = theta3 * theta3, theta7 = theta6 * theta,
        theta8 = theta4 * theta4, theta9 = theta8 * theta;
      T theta_d = theta + k1 * theta3 + k2 * theta5 + p1 * theta7 + p2 * theta9;
      T inv_r = r > T(1e-8) ? T(1.0) / r : T(1);
      T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);
      //    T inv_r =  T(1)/r;
      //    T cdist = theta_d * inv_r
      T xd = pobj[0] * cdist;
      T yd = pobj[1] * cdist;

      // Project on the image plane
      T up = T(focal_x) * xd + T(u0);
      T vp = T(focal_y) * yd + T(v0);

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }
    // T error = sqrt((residuals[0] + residuals[1])*(residuals[0] +
    // residuals[1])); std::cout << "residual " <<  error <<  std::endl;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *
  Create(const double u, const double v, const double x, const double y,
         const double z, const double focal_x, const double focal_y,
         const double u0, const double v0, const double k1, const double k2,
         const double k3, const double p1, const double p2,
         const bool refine_camera, const int distortion_type) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError_CameraGroupRef, 2,
                                            6, 6>(
        new ReprojectionError_CameraGroupRef(u, v, x, y, z, focal_x, focal_y,
                                             u0, v0, k1, k2, k3, p1, p2,
                                             refine_camera, distortion_type)));
  }

  double u, v;
  double x;
  double y;
  double z;
  double focal_x;
  double focal_y;
  double u0;
  double v0;
  double k1;
  double k2;
  double k3;
  double p1;
  double p2;
  bool refine_camera;
  int distortion_type;
};

// Refine camera group (3D object pose + relative camera pose + Board pose)
struct ReprojectionError_CameraGroupAndObjectRef {
  ReprojectionError_CameraGroupAndObjectRef(
      double u, double v, double x, double y, double z, double focal_x,
      double focal_y, double u0, double v0, double k1, double k2, double k3,
      double p1, double p2, bool refine_camera, bool refine_board,
      int distortion_type)
      : u(u), v(v), x(x), y(y), z(z), focal_x(focal_x), focal_y(focal_y),
        u0(u0), v0(v0), k1(k1), k2(k2), k3(k3), p1(p1), p2(p2),
        refine_camera(refine_camera), refine_board(refine_board),
        distortion_type(distortion_type) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const object_pose,
                  const T *const board_pose, T *residuals) const {

    // 1. Apply the board transformation in teh object
    T point[3] = {T(x), T(y), T(z)};
    if (refine_board != 0) {
      T point_refine[3];
      ceres::AngleAxisRotatePoint(board_pose, point, point_refine);
      std::copy_n(point_refine, 3, point);
      point[0] += board_pose[3];
      point[1] += board_pose[4];
      point[2] += board_pose[5];
    }

    // 2. apply transformation to the object (to expressed in the current
    // camera)
    T pobj[3];
    ceres::AngleAxisRotatePoint(object_pose, point, pobj);
    pobj[0] += object_pose[3];
    pobj[1] += object_pose[4];
    pobj[2] += object_pose[5];

    // 3. Refine the camera if it is not the referential
    if (refine_camera != 0) {
      T pobj_refine[3];
      ceres::AngleAxisRotatePoint(camera, pobj, pobj_refine);
      std::copy_n(pobj_refine, 3, pobj);
      pobj[0] += camera[3];
      pobj[1] += camera[4];
      pobj[2] += camera[5];
    }

    // Normalization on the camera plane
    pobj[0] /= pobj[2];
    pobj[1] /= pobj[2];

    if (distortion_type == 0) // perspective brown
    {
      // apply distorsion
      T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
      T r4 = r2 * r2;
      T r6 = r4 * r2;
      T r_coeff = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);
      T xd = pobj[0] * r_coeff + T(2) * p1 * pobj[0] * pobj[1] +
             p2 * (r2 + T(2) * pobj[0] * pobj[0]);
      T yd = pobj[1] * r_coeff + p1 * (r2 + T(2) * (pobj[1] * pobj[1])) +
             T(2) * p2 * pobj[0] * pobj[1];

      // Project on the image plane
      T up = T(focal_x) * xd + T(u0);
      T vp = T(focal_y) * yd + T(v0);

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }

    if (distortion_type == 1) // fisheye
    {
      // apply distorsion
      // (source : https://www.programmersought.com/article/72251092167/)

      T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
      using std::atan;
      using std::sqrt;
      T r = sqrt(r2);
      // auto r=sqrt<T>(r2);
      T theta = atan(r);
      // T theta = atan<T>(r);
      T theta2 = theta * theta, theta3 = theta2 * theta,
        theta4 = theta2 * theta2, theta5 = theta4 * theta;
      T theta6 = theta3 * theta3, theta7 = theta6 * theta,
        theta8 = theta4 * theta4, theta9 = theta8 * theta;
      T theta_d = theta + k1 * theta3 + k2 * theta5 + p1 * theta7 + p2 * theta9;
      T inv_r = r > T(1e-8) ? T(1.0) / r : T(1);
      T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);
      //    T inv_r =  T(1)/r;
      //    T cdist = theta_d * inv_r
      T xd = pobj[0] * cdist;
      T yd = pobj[1] * cdist;

      // Project on the image plane
      T up = T(focal_x) * xd + T(u0);
      T vp = T(focal_y) * yd + T(v0);

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }
    // T error = sqrt((residuals[0] + residuals[1])*(residuals[0] +
    // residuals[1])); std::cout << "residual " <<  error <<  std::endl;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *
  Create(const double u, const double v, const double x, const double y,
         const double z, const double focal_x, const double focal_y,
         const double u0, const double v0, const double k1, const double k2,
         const double k3, const double p1, const double p2,
         const bool refine_camera, const bool refine_board,
         const int distortion_type) {
    return (new ceres::AutoDiffCostFunction<
            ReprojectionError_CameraGroupAndObjectRef, 2, 6, 6, 6>(
        new ReprojectionError_CameraGroupAndObjectRef(
            u, v, x, y, z, focal_x, focal_y, u0, v0, k1, k2, k3, p1, p2,
            refine_camera, refine_board, distortion_type)));
  }

  double u, v;
  double x;
  double y;
  double z;
  double focal_x;
  double focal_y;
  double u0;
  double v0;
  double k1;
  double k2;
  double k3;
  double p1;
  double p2;
  bool refine_camera;
  bool refine_board;
  int distortion_type;
};

// Refine camera group (3D object pose + relative camera pose + Board pose)
struct ReprojectionError_CameraGroupAndObjectRefAndIntrinsics {
  ReprojectionError_CameraGroupAndObjectRefAndIntrinsics(
      double u, double v, double x, double y, double z, bool refine_camera,
      bool refine_board, int distortion_type)
      : u(u), v(v), x(x), y(y), z(z), refine_camera(refine_camera),
        refine_board(refine_board), distortion_type(distortion_type) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const object_pose,
                  const T *const board_pose, const T *const cam_int,
                  T *residuals) const {

    // 0. prepare intrinsics
    T focal_x = cam_int[0];
    T focal_y = cam_int[1];
    T u0 = cam_int[2];
    T v0 = cam_int[3];
    T k1 = cam_int[4];
    T k2 = cam_int[5];
    T k3 = cam_int[8];
    T p1 = cam_int[6];
    T p2 = cam_int[7];

    // 1. Apply the board transformation in teh object
    T point[3] = {T(x), T(y), T(z)};
    if (refine_board != 0) {
      T point_refine[3];
      ceres::AngleAxisRotatePoint(board_pose, point, point_refine);
      std::copy_n(point_refine, 3, point);
      point[0] += board_pose[3];
      point[1] += board_pose[4];
      point[2] += board_pose[5];
    }

    // 2. apply transformation to the object (to expressed in the current
    // camera)
    T pobj[3];
    ceres::AngleAxisRotatePoint(object_pose, point, pobj);
    pobj[0] += object_pose[3];
    pobj[1] += object_pose[4];
    pobj[2] += object_pose[5];

    // 3. Refine the camera if it is not the referential
    if (refine_camera != 0) {
      T pobj_refine[3];
      ceres::AngleAxisRotatePoint(camera, pobj, pobj_refine);
      std::copy_n(pobj_refine, 3, pobj);
      pobj[0] += camera[3];
      pobj[1] += camera[4];
      pobj[2] += camera[5];
    }

    // Normalization on the camera plane
    pobj[0] /= pobj[2];
    pobj[1] /= pobj[2];

    if (distortion_type == 0) // perspective brown
    {
      // apply distorsion
      T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
      T r4 = r2 * r2;
      T r6 = r4 * r2;
      T r_coeff = (T(1) + k1 * r2 + k2 * r4 + k3 * r6);
      T xd = pobj[0] * r_coeff + T(2) * p1 * pobj[0] * pobj[1] +
             p2 * (r2 + T(2) * pobj[0] * pobj[0]);
      T yd = pobj[1] * r_coeff + p1 * (r2 + T(2) * (pobj[1] * pobj[1])) +
             T(2) * p2 * pobj[0] * pobj[1];

      // Project on the image plane
      T up = T(focal_x) * xd + T(u0);
      T vp = T(focal_y) * yd + T(v0);

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }

    if (distortion_type == 1) // fisheye
    {

      // apply distorsion
      // (source : https://www.programmersought.com/article/72251092167/)

      T r2 = pobj[0] * pobj[0] + pobj[1] * pobj[1];
      using std::atan;
      using std::sqrt;
      T r = sqrt(r2);
      // auto r=sqrt<T>(r2);
      T theta = atan(r);
      // T theta = atan<T>(r);
      T theta2 = theta * theta, theta3 = theta2 * theta,
        theta4 = theta2 * theta2, theta5 = theta4 * theta;
      T theta6 = theta3 * theta3, theta7 = theta6 * theta,
        theta8 = theta4 * theta4, theta9 = theta8 * theta;
      T theta_d = theta + k1 * theta3 + k2 * theta5 + p1 * theta7 + p2 * theta9;
      T inv_r = r > T(1e-8) ? T(1.0) / r : T(1);
      T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);
      //    T inv_r =  T(1)/r;
      //    T cdist = theta_d * inv_r
      T xd = pobj[0] * cdist;
      T yd = pobj[1] * cdist;

      // Project on the image plane
      T up = T(focal_x) * xd + T(u0);
      T vp = T(focal_y) * yd + T(v0);

      // The error is the difference between the predicted and observed
      // position.
      residuals[0] = up - T(u);
      residuals[1] = vp - T(v);
    }
    // T error = sqrt((residuals[0] + residuals[1])*(residuals[0] +
    // residuals[1])); std::cout << "residual " <<  error <<  std::endl;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const double u, const double v,
                                     const double x, const double y,
                                     const double z, const bool refine_camera,
                                     const bool refine_board,
                                     const int distortion_type) {
    return (new ceres::AutoDiffCostFunction<
            ReprojectionError_CameraGroupAndObjectRefAndIntrinsics, 2, 6, 6, 6,
            9>(new ReprojectionError_CameraGroupAndObjectRefAndIntrinsics(
        u, v, x, y, z, refine_camera, refine_board, distortion_type)));
  }

  double u, v;
  double x;
  double y;
  double z;
  bool refine_camera;
  bool refine_board;
  int distortion_type;
};

/*
template <typename T>
Eigen::Matrix<T, 3, 3>
AngleAxisToRotationMatrix(const Eigen::Matrix<T, 3, 1>& rvec) {
    T angle = rvec.norm();
    if (angle == T(0)) {
        return Eigen::Matrix<T, 3, 3>::Identity();
    }

    Eigen::Matrix<T, 3, 1> axis;
    axis = rvec.normalized();

    Eigen::Matrix<T, 3, 3> rmat;
    rmat = Eigen::AngleAxis<T>(angle, axis);

    return rmat;
}



// Refine handeye calibration
struct TransformationError {
  TransformationError(double r1_x, double r1_y, double r1_z, double t1_x, double
t1_y, double t1_z, double r2_x, double r2_y, double r2_z, double t2_x, double
t2_y, double t2_z) : r1_x(r1_x), r1_y(r1_y), r1_z(r1_z), t1_x(t1_x), t1_y(t1_y),
t1_z(t1_z), r2_x(r2_x), r2_y(r2_y), r2_z(r2_z), t2_x(t2_x), t2_y(t2_y),
t2_z(t2_z)  {}

  template <typename T>
  bool operator()(const T *const transform,
                  T *residuals) const {

    // 1. convert to rotation matrix for relative 1 and 2
    Eigen::Matrix<double, 3, 1> r1, r2;
    r1(0,0) = r1_x; r1(1,0) = r1_y; r1(2,0) = r1_z;
    r2(0,0) = r2_x; r2(1,0) = r2_y; r2(2,0) = r2_z;
    Eigen::Matrix<double, 3, 3> R1 = AngleAxisToRotationMatrix(r1);
    Eigen::Matrix<double, 3, 3> R2 = AngleAxisToRotationMatrix(r2);
    Eigen::Matrix<double, 3,1> t1, t2;
    t1(0,0) = t1_x; t1(1,0) = t1_y; t1(2,0) = t1_z;
    t2(0,0) = t2_x; t2(1,0) = t2_y; t2(2,0) = t2_z;

    Eigen::Matrix4d Trans1; // Your Transformation Matrix
    Trans1.setIdentity();   // Set to Identity to make bottom row of Matrix
0,0,0,1 Trans1.block<3,3>(0,0) = R1; Trans1.block<3,1>(0,3) = t1;
    Eigen::Matrix4d Trans2; // Your Transformation Matrix
    Trans2.setIdentity();   // Set to Identity to make bottom row of Matrix
0,0,0,1 Trans2.block<3,3>(0,0) = R2; Trans2.block<3,1>(0,3) = t2;

    //2. convert the transform
    Eigen::Matrix<T, 3, 1> rt;
    Eigen::Matrix<T, 3,1> tt;
    rt(0,0) = (transform[0]); rt(1,0) = (transform[1]); rt(2,0) =
(transform[2]); Eigen::Matrix<T, 3, 3> Rt = AngleAxisToRotationMatrix(rt);
    tt(0,0) = (transform[3]); tt(1,0) = (transform[4]); tt(2,0) =
(transform[5]); Eigen::Matrix<T, 4,4> Trans; // Your Transformation Matrix
    Trans(0,0) = Rt(0,0); Trans(0,1) = Rt(0,1); Trans(0,2) = Rt(0,2); Trans(0,3)
= tt(0,0); Trans(1,0) = Rt(1,0); Trans(1,1) = Rt(1,1); Trans(1,2) = Rt(1,2);
Trans(1,3) = tt(1,0); Trans(2,0) = Rt(2,0); Trans(2,1) = Rt(2,1); Trans(2,2) =
Rt(2,2); Trans(2,3) = tt(2,0);

    Eigen::Matrix4d Error = (Trans - Trans2*Trans*Trans1.inverse());
    //r1.row(0) << r1_x, r1_y, r1_z;
    //Map<Vector3f> r1; //(r11);


    //AngleAxisToRotationMatrix(axis_angle, matrix);
    // 2. convert the "transform" ro rotation matrix

    // 2. Compute errors matrix
    residuals[0] = T(Error(0,0));
    residuals[1] = T(Error(1,0));
    residuals[2] = T(Error(2,0));
    residuals[3] = T(Error(3,0));
    residuals[4] = T(Error(0,1));
    residuals[5] = T(Error(1,1));
    residuals[6] = T(Error(2,1));
    residuals[7] = T(Error(3,1));
    residuals[8] = T(Error(0,2));
    residuals[9] = T(Error(1,2));
    residuals[10] = T(Error(2,2));
    residuals[11] = T(Error(3,2));

    return true;
  }

  static ceres::CostFunction *Create(const double r1_x, const double r1_y, const
double r1_z, const double t1_x, const double t1_y, const double t1_z, const
double r2_x, const double r2_y, const double r2_z, const double t2_x, const
double t2_y, const double t2_z) { return (new
ceres::AutoDiffCostFunction<TransformationError, 12, 6>( new
TransformationError(r1_x, r1_y, r1_z, t1_x, t1_y, t1_z, r2_x, r2_y, r2_z, t2_x,
t2_y, t2_z)));
  }
  double r1_x, r1_y, r1_z;
  double t1_x, t1_y, t1_z;
  double r2_x, r2_y, r2_z;
  double t2_x, t2_y, t2_z;
};*/