/**
 * @brief Saddle Point Refinement
 *
 * Adapted from
 * https://github.com/facebookarchive/deltille/blob/master/include/deltille/PolynomialFit.h
 *
 * Reference: "Deltille Grids for Geometric Camera Calibration", H Ha, M
 * Perdoch, H Alismail, I So Kweon, Y Sheikh (ICCV 2017)
 */

#include <cmath>
#include <iostream>
#include <stdio.h>

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>

namespace McCalib {

struct SaddlePoint {
  double x;
  double y;
  double a1;
  double a2;
  double s;
  double det;
  SaddlePoint() {}
  SaddlePoint(double x, double y) : x(x), y(y) {}
};

void initConeSmoothingKernel(const int window_half_size,
                             cv::Mat &smoothingKernel, cv::Mat &mask,
                             int &nnz) {
  int window_size = window_half_size * 2 + 1;
  smoothingKernel.create(window_size, window_size, cv::DataType<float>::depth);
  mask.create(window_size, window_size, CV_8UC1);
  double maxVal = window_half_size + 1, sum = 0;
  float *w = smoothingKernel.ptr<float>(0);
  // cone kernel
  for (int y = -window_half_size; y <= window_half_size; y++)
    for (int x = -window_half_size; x <= window_half_size; x++) {
      *w = float(maxVal - std::sqrt(x * x + y * y));
      if (*w > 0)
        nnz++;
      else
        *w = 0;
      sum += *w;
      w++;
    }
  // scale kernel
  smoothingKernel /= sum;
}

void initSaddleFitting(const int half_kernel_size, const int nnz,
                       cv::Mat &smoothingKernel, cv::Mat &mask,
                       cv::Mat &invAtAAt) {
  cv::Mat A(nnz, 6, CV_64FC1);
  double *a = A.ptr<double>(0);
  float *w = smoothingKernel.ptr<float>(0);
  uint8_t *m = mask.ptr<uint8_t>(0);
  for (int y = -half_kernel_size; y <= half_kernel_size; y++)
    for (int x = -half_kernel_size; x <= half_kernel_size; x++) {
      if (*w > 0) {
        *m = 255;
        a[0] = x * x;
        a[1] = y * y;
        a[2] = x * y;
        a[3] = y;
        a[4] = x;
        a[5] = 1;
        a += 6;
      } else
        *m = 0;
      w++;
      m++;
    }
  // compute pseudoinverse of AtA
  cv::invert(A.t() * A, invAtAAt, cv::DECOMP_SVD);
  invAtAAt *= A.t();
}

template <typename ImageType>
bool interpolatePatch(double x, double y, int window_half_size,
                      const cv::Mat &input, const cv::Mat &mask,
                      cv::Mat &b_vec) {
  if (x > window_half_size + 1 && x < input.cols - (window_half_size + 2) &&
      y > window_half_size + 1 && y < input.rows - (window_half_size + 2)) {
    int x0 = int(x);
    int y0 = int(y);
    double xw = x - x0;
    double yw = y - y0;

    // precompute bilinear interpolation weights
    double w00 = (1.0 - xw) * (1.0 - yw), w01 = xw * (1.0 - yw),
           w10 = (1.0 - xw) * yw, w11 = xw * yw;

    // fit to local neighborhood = b vector...
    const uint8_t *v = mask.ptr<const uint8_t>(0);
    double *m = b_vec.ptr<double>(0);
    double mn = DBL_MAX;
    double mx = DBL_MIN;
    for (int wy = -window_half_size; wy <= window_half_size; wy++) {
      const ImageType *im00 = input.ptr<ImageType>(y0 + wy),
                      *im10 = input.ptr<ImageType>(y0 + wy + 1);
      for (int wx = -window_half_size; wx <= window_half_size; wx++) {
        if (*v > 0) {
          const int col0 = x0 + wx;
          const int col1 = col0 + 1;
          double val = im00[col0] * w00 + im00[col1] * w01 + im10[col0] * w10 +
                       im10[col1] * w11;
          *(m++) = val;
          if (mn > val)
            mn = val;
          if (mx < val)
            mx = val;
        }
        v++;
      }
    }
    if (mx - mn > 1.0 / 255)
      return true;
  }
  return false;
}

template <typename LocationsPointType, typename SmoothedImageType = double>
void saddleSubpixelRefinement(
    const cv::Mat &smoothed_input,
    const std::vector<LocationsPointType> &initial, const cv::Mat &invAtAAt,
    const cv::Mat &mask, const int window_half_size,
    std::vector<SaddlePoint> &refined, const int max_iterations = 3,
    const bool tight_convergence = true,
    const double spatial_convergence_threshold = 0.001) {
  cv::Mat b(invAtAAt.cols, 1, CV_64FC1);

  double convergence_region = window_half_size;
  if (tight_convergence) {
    convergence_region = 1.0;
  }

  refined.resize(initial.size());
  for (size_t idx = 0; idx < initial.size(); idx++)
    refined[idx] = SaddlePoint(initial[idx].x, initial[idx].y);

  for (size_t idx = 0; idx < refined.size(); idx++) {
    SaddlePoint &pt = refined[idx];
    bool point_diverged = true;
    for (int it = 0; it < max_iterations; it++) {
      if (interpolatePatch<SmoothedImageType>(pt.x, pt.y, window_half_size,
                                              smoothed_input, mask, b)) {
        // fit quadric to surface by solving LSQ
        cv::Mat p = invAtAAt * b;

        // k5, k4, k3, k2, k1, k0
        // 0 , 1 , 2 , 3 , 4 , 5
        double *r = p.ptr<double>(0);
        // 4.0 * k5 * k4 - k3 * k3
        pt.det = 4.0 * r[0] * r[1] - r[2] * r[2];

        // check if it is still a saddle point
        if (pt.det > 0) {
          break;
        }

        // compute the new location
        // - 2 * k4 * k1 +     k3 * k2
        double dx = (-2.0 * r[1] * r[4] + r[2] * r[3]) / pt.det;
        // k3 * k1 - 2 * k5 * k2
        double dy = (r[2] * r[4] - 2.0 * r[0] * r[3]) / pt.det;
        pt.x += dx;
        pt.y += dy;

        if (spatial_convergence_threshold > std::fabs(dx) &&
            spatial_convergence_threshold > std::fabs(dy)) {
          double k4mk5 = r[1] - r[0];
          pt.s = std::sqrt(r[2] * r[2] + k4mk5 * k4mk5);
          pt.a1 = std::atan2(-r[2], k4mk5) / 2.0;
          pt.a2 = std::acos((r[1] + r[0]) / pt.s) / 2.0;
          // converged
          point_diverged = false;
          break;
        }
        // check for divergence due to departure out of convergence region or
        // point type change
        if (std::fabs(pt.x - initial[idx].x) > convergence_region ||
            std::fabs(pt.y - initial[idx].y) > convergence_region) {
          break;
        }
      } else {
        break;
      }
    }
    if (point_diverged) {
      pt.x = pt.y = std::numeric_limits<double>::infinity();
    }
  }
}

template <typename PointType>
void saddleSubpixelRefinement(const cv::Mat &input,
                              const std::vector<PointType> &initial,
                              std::vector<SaddlePoint> &refined,
                              const int window_half_size = 2,
                              const int max_iterations = 20) {

  // LOG_INFO << "Saddle point refinement";

  cv::Mat smoothingKernel;
  cv::Mat mask;
  int nnz;
  initConeSmoothingKernel(window_half_size, smoothingKernel, mask, nnz);

  // LOG_INFO << "initConeSmoothingKernel() done";

  cv::Mat invAtAAt;
  initSaddleFitting(window_half_size, nnz, smoothingKernel, mask, invAtAAt);

  // LOG_INFO << "initSaddleFitting() done";

  cv::Mat smooth_input;
  cv::filter2D(input, smooth_input, CV_64FC1, smoothingKernel);

  saddleSubpixelRefinement(smooth_input, initial, invAtAAt, mask,
                           window_half_size, refined, max_iterations);

  // LOG_INFO << "saddleSubpixelRefinement() done";
}

} // namespace McCalib