#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <cmath>
#include <eigen3/Eigen/Eigen>

namespace spline {
void spline(const Eigen::VectorXd& x, Eigen::VectorXd& a, Eigen::VectorXd& b,
            Eigen::VectorXd& c, Eigen::VectorXd& d) {
  int n = x.size() - 1;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n + 1),
                  B = Eigen::MatrixXd::Zero(n, n + 1),
                  C = Eigen::MatrixXd::Zero(n, n + 1),
                  D = Eigen::MatrixXd::Zero(n, n + 1);
  A(0, 0) = 2, A(0, 1) = -3, A(0, 2) = 1;
  B(0, 0) = -3, B(0, 1) = 4, B(0, 2) = -1;
  D(0, 0) = 1;
  for (int i = 1; i < n - 1; ++i) {
    A(i, i) = 1, A(i, i + 1) = -2, A(i, i + 2) = 1;
    B(i, i) = -1, B(i, i + 1) = 2, B(i, i + 2) = -1;
    C(i, i) = -1, C(i, i + 1) = 1;
    D(i, i) = 1;
  }
  A(n - 1, n - 1) = 1, A(n - 1, n) = -1;
  B(n - 1, n - 1) = -1, B(n - 1, n) = 1;
  C(n - 1, n - 1) = -1, C(n - 1, n) = 1;
  D(n - 1, n - 1) = 1;
  a = A * x;
  b = B * x;
  c = C * x;
  d = D * x;
  return;
}
}  // namespace spline
#endif