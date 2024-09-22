#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>

namespace McCalib {

/****** ******/

float step_threshold = 0.001;

/**
 * @struct SaddlePoint
 *
 * @brief Saddle Point Refinement
 *
 * Reference: "Deltille Grids for Geometric Camera Calibration", H Ha, M
 * Perdoch, H Alismail, I So Kweon, Y Sheikh (ICCV 2017)
 */
struct SaddlePoint {
  double x, y;
  double a1, a2;
  double s, det;
  SaddlePoint() {}
  SaddlePoint(double x, double y) : x(x), y(y) {}
};

void initSaddlePointRefinement(int half_kernel_size, cv::Mat &saddleKernel,
                               cv::Mat &invAtAAt, cv::Mat &valid) {
  int window_size = half_kernel_size * 2 + 1;
  saddleKernel.create(window_size, window_size, CV_64FC1);
  double maxVal = half_kernel_size + 1, sum = 0;
  double *w = saddleKernel.ptr<double>(0);
  // circular kernel
  int cnt = 0;
  for (int y = -half_kernel_size; y <= half_kernel_size; y++)
    for (int x = -half_kernel_size; x <= half_kernel_size; x++) {
      *w = maxVal - sqrt(x * x + y * y);
      if (*w > 0)
        cnt++;
      else
        *w = 0;
      sum += *w;
      w++;
    }
  // scale kernel
  saddleKernel /= sum;

  cv::Mat A(cnt, 6, CV_64FC1);
  double *a = A.ptr<double>(0);
  w = saddleKernel.ptr<double>(0);
  valid.create(window_size, window_size, CV_8UC1);
  uint8_t *v = valid.ptr<uint8_t>(0);

  for (int y = -half_kernel_size; y <= half_kernel_size; y++)
    for (int x = -half_kernel_size; x <= half_kernel_size; x++) {
      if (*w > 0) {
        *v = 255;
        a[0] = x * x;
        a[1] = y * y;
        a[2] = x * y;
        a[3] = y;
        a[4] = x;
        a[5] = 1;
        a += 6;
      } else {
        *v = 0;
      }
      w++;
      v++;
    }
  // compute pseudoinverse of AtA
  cv::invert(A.t() * A, invAtAAt, cv::DECOMP_SVD);
  invAtAAt *= A.t();
}

template <typename PointType>
void saddleSubpixelRefinement(const cv::Mat &smooth_input,
                              const std::vector<PointType> &initial, cv::Mat &A,
                              cv::Mat &valid, std::vector<SaddlePoint> &refined,
                              int window_half_size = 3,
                              int max_iterations = 3) {
  cv::Mat b(A.cols, 1, CV_64FC1);

  refined.resize(initial.size());
  for (size_t idx = 0; idx < initial.size(); idx++)
    refined[idx] = SaddlePoint(initial[idx].x, initial[idx].y);

  // int diverged = 0, iterations = 0;
  int width = smooth_input.cols, height = smooth_input.rows;
  for (size_t idx = 0; idx < refined.size(); idx++) {
    SaddlePoint &pt = refined[idx];
    // printf("initial: %.3f, %.3f: ", pt.x, pt.y);
    cv::Mat p;
    for (int it = 0; it < max_iterations; it++) {
      if (pt.x > window_half_size + 1 &&
          pt.x < width - (window_half_size + 2) &&
          pt.y > window_half_size + 1 &&
          pt.y < height - (window_half_size + 2)) {
        int x0 = int(pt.x);
        int y0 = int(pt.y);
        double xw = pt.x - x0;
        double yw = pt.y - y0;

        // precompute bilinear interpolation weights
        double w00 = (1.0 - xw) * (1.0 - yw), w01 = xw * (1.0 - yw),
               w10 = (1.0 - xw) * yw, w11 = xw * yw;

        // fit to local neighborhood = b vector...
        double *m = b.ptr<double>(0);
        uint8_t *v = valid.ptr<uint8_t>(0);

        for (int y = -window_half_size; y <= window_half_size; y++) {
          const double *im00 = smooth_input.ptr<double>(y0 + y),
                       *im10 = smooth_input.ptr<double>(y0 + y + 1);
          for (int x = -window_half_size; x <= window_half_size; x++) {
            if (*v > 0) {
              const int col0 = x0 + x;
              const int col1 = col0 + 1;
              *(m++) = im00[col0] * w00 + im00[col1] * w01 + im10[col0] * w10 +
                       im10[col1] * w11;
            }
            v++;
          }
        }
        // fit quadric to surface by solving LSQ
        p = A * b;

        // k5, k4, k3, k2, k1, k0
        // 0 , 1 , 2 , 3 , 4 , 5
        double *r = p.ptr<double>(0);
        pt.det = 4.0 * r[0] * r[1] - r[2] * r[2]; // 4.0 * k5 * k4 - k3 * k3
                                                  // compute the new location
        double dx = (-2 * r[1] * r[4] + r[2] * r[3]) /
                    pt.det; // - 2 * k4 * k1 +     k3 * k2
        double dy = (r[2] * r[4] - 2 * r[0] * r[3]) /
                    pt.det; //       k3 * k1 - 2 * k5 * k2
        pt.x += dx;
        pt.y += dy;
        dx = fabs(dx);
        dy = fabs(dy);
        // iterations++;

        if (it == max_iterations ||
            (step_threshold > dx && step_threshold > dy)) {
          // converged
          double k4mk5 = r[1] - r[0];
          pt.s = sqrt(r[2] * r[2] + k4mk5 * k4mk5);
          pt.a1 = atan2(-r[2], k4mk5) / 2.0;
          pt.a2 = acos((r[1] + r[0]) / pt.s) / 2.0;
          break;
        } else {
          // check for divergence
          if (pt.det > 0 || fabs(pt.x - initial[idx].x) > window_half_size ||
              fabs(pt.y - initial[idx].y) > window_half_size) {
            pt.x = pt.y = std::numeric_limits<double>::infinity();
            // diverged++;
            break;
          }
        }
      } else {
        // too close to border...
        pt.x = pt.y = std::numeric_limits<double>::infinity();
        // diverged++;
        break;
      }
    }
    // printf(" [%.3f, %.3f]\n", pt.x, pt.y);
  }
  // printf("Total: %zd, diverged: %d, iterations: %d\n", initial.size(),
  // diverged, iterations);
}

template <typename PointType>
void saddleSubpixelRefinement(const cv::Mat &input,
                              const std::vector<PointType> &initial,
                              std::vector<SaddlePoint> &refined,
                              int window_half_size = 2,
                              int max_iterations = 20) {
  cv::Mat weights, A, valid;
  initSaddlePointRefinement(window_half_size, weights, A, valid);

  // circular filter smoothing
  cv::Mat smooth;
  cv::filter2D(input, smooth, CV_64FC1, weights);

  saddleSubpixelRefinement(smooth, initial, A, valid, refined, window_half_size,
                           max_iterations);
}

} // namespace McCalib