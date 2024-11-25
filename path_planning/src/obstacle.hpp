#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

#include "utils/sdqp.hpp"

/**
 * @brief Obstacle class
 * @param n dimension of the space
 * @param m number of inequality constraints
 */
template <int n>
class Obstacle {
 private:
  // A * x <= b
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  int m;

 public:
  Obstacle(int m, Eigen::MatrixXd A, Eigen::VectorXd b) {
    this->A = A;
    this->b = b;
    this->m = m;
    // this->A.resize(m, n);
    // this->b.resize(m);
  }

  inline double distance(const Eigen::Matrix<double, n, 1>& x) {
    if (inside(x)) {
      double distance = 0x3f3f3f3f;
      for (int i = 0; i < A.rows(); ++i) {
        // 1e-16 for numerical stability
        distance = std::min(distance,
                            (b(i) - A.row(i) * x) / (A.row(i).norm() + 1e-16));
      }
      return distance;
    }
    if (m == 1) {
      return -abs((A * x - b)(0)) / (A.row(0).norm() + 1e-16);
    }
    Eigen::Matrix<double, n, 1> xbar = x;
    sdqp::sdqp<n>(2 * Eigen::Matrix<double, n, n>::Identity(), -2 * x, A, b,
                  xbar);
    return -(x - xbar).norm();
  }

  inline bool inside(const Eigen::Matrix<double, n, 1>& x) {
    return (A * x - b).maxCoeff() <= 0;
  }
};

#endif  // OBSTACLE_HPP