#ifndef SMOOTH_HPP
#define SMOOTH_HPP

#include <ros/ros.h>

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "config.h"
#include "map.hpp"
#include "utils/spline.hpp"
class Smooth {
 private:
  // Eigen::VectorXd last_grad;
  // Eigen::MatrixXd B;
  int n, iter;

  // int check(const Eigen::VectorXd &var) {
  //   int cnt = 0;
  //   for (int i = 0; i < n; ++i) {
  //     if (Map::getInstance()->distance(Eigen::Vector2d(var(i), var(i + n))) <
  //         -0.5) {
  //       cnt++;
  //     }
  //   }
  //   return cnt;
  // }

  // double energy(const Eigen::VectorXd &x, const Eigen::VectorXd &y) {
  //   double res = 0;
  //   Eigen::VectorXd ax, bx, cx, dx, ay, by, cy, dy;
  //   spline::spline(x, ax, bx, cx, dx);
  //   spline::spline(y, ay, by, cy, dy);
  //   // integration of the absolute value of acceleration
  //   for (int i = 0; i < ax.size(); i++) {
  //     res += abs(3 * ax(i) + bx(i)) + abs(bx(i)) + abs(3 * ay(i) + by(i)) +
  //            abs(by(i));
  //   }
  //   return res;
  // }

  double energy(const std::vector<Eigen::Vector2d> &points) {
    double res = 0;
    for (int i = 1; i < points.size() - 1; i++) {
      res += (points[i - 1] - 2 * points[i] + points[i + 1]).norm();
    }
    return res;
  }

  double potential(const std::vector<Eigen::Vector2d> &points) {
    double res = 0;
    Map *map = Map::getInstance();
    for (auto &point : points) {
      // ROS_INFO("point: %f, %f", point(0), point(1));
      // ROS_INFO("inside: %d", map->occupied(point));
      // ROS_INFO("distance: %f", map->distance(point));
      res += exp(4 * map->distance(point));
    }
    return res;
  }

  double loss(const Eigen::VectorXd &var) {
    Eigen::VectorXd x = var.head(n), y = var.tail(n);
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i < n; i++) {
      points.push_back(Eigen::Vector2d(x(i), y(i)));
    }
    return potential(points) + energy(points);
  }

  Eigen::VectorXd gradient(const Eigen::VectorXd &var) {
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(2 * n);
    for (int i = 0; i < 2 * n; i++) {
      if (i == 0 || i == n - 1 || i == n || i == 2 * n - 1) {
        continue;
      }
      Eigen::VectorXd bigger = var, smaller = var;
      bigger(i) += 5e-9;
      smaller(i) -= 5e-9;
      grad(i) = (loss(bigger) - loss(smaller)) / 1e-8;
    }
    return grad;
  }

 public:
  Smooth() {}
  Smooth(int n) {
    this->n = n;
    iter = 0;
    // last_grad = Eigen::VectorXd::Zero(2 * n);
    // B = Eigen::MatrixXd::Identity(2 * n, 2 * n);
  }

  bool step(std::vector<Eigen::Vector2d> &points, Eigen::VectorXd &direction) {
    ++iter;
    Eigen::VectorXd var(2 * n);
    for (int i = 0; i < n; i++) {
      var(i) = points[i](0) * PATH_CELL_SIZE - MAP_SIZE / 2;
      var(i + n) = points[i](1) * PATH_CELL_SIZE - MAP_SIZE / 2;
    }
    Eigen::VectorXd grad = gradient(var);
    // Eigen::VectorXd d = -B * grad;
    // // line search
    // double alpha = 0.0005;
    // // while (!(loss(var) - loss(var + alpha * d) >= -alpha * 1e-4 *
    // // d.dot(grad)
    // // &&
    // //          d.dot(gradient(var + alpha * d)) >= 0.9 * d.dot(grad))) {
    // //   alpha *= .5;
    // // }
    // direction = d;
    // Eigen::VectorXd var_new = var + alpha * d;
    // Eigen::VectorXd grad_new = gradient(var_new);
    // // cautious update
    // if ((grad_new - grad).dot(var_new - var) > 1e-6 * grad.norm()) {
    //   B = (Eigen::MatrixXd::Identity(2 * n, 2 * n) -
    //        (var_new - var) * (grad_new - grad).transpose() /
    //            (grad_new - grad).dot(var_new - var)) *
    //           B *
    //           (Eigen::MatrixXd::Identity(2 * n, 2 * n) -
    //            (grad_new - grad) * (var_new - var).transpose() /
    //                (grad_new - grad).dot(var_new - var)) +
    //       (var_new - var) * (var_new - var).transpose() /
    //           (grad_new - grad).dot(var_new - var);
    // }

    Eigen::VectorXd d = -grad;
    for (int i = 0; i < n; i++) {
      // if (rand() % DROPOUT != 0) {
      //   d(i) = 1 / (1 + exp(-d(i))) - 0.5;
      // }
      // if (rand() % DROPOUT != 0) {
      //   d(i + n) = 1 / (1 + exp(-d(i + n))) - 0.5;
      // }
      d(i) = 1 / (1 + exp(-d(i))) - 0.5;
      d(i + n) = 1 / (1 + exp(-d(i + n))) - 0.5;
    }
    d *= 2;
    if (iter < MAX_ITER / 5) {
      d *= LEARNING_RATE1;
    } else if (iter / 2 < MAX_ITER / 5) {
      d *= LEARNING_RATE2;
    } else if (iter / 3 < MAX_ITER / 5) {
      d *= LEARNING_RATE3;
    } else if (iter / 4 < MAX_ITER / 5) {
      d *= LEARNING_RATE4;
    } else {
      d *= LEARNING_RATE5;
    }
    Eigen::VectorXd var_new = var + d;
    Eigen::VectorXd grad_new = gradient(var_new);
    direction = (var_new - var) * 33;
    // ROS_INFO("iter: %d/%d, loss: %f, grad: %f", iter, MAX_ITER,
    // loss(var_new), grad_new.norm());
    // if (loss(var_new) > n * 3e-1 + loss(var)) {
    //   ROS_ERROR("optimization not converged");
    //   return true;
    // }
    if (iter >= MAX_ITER) {
      ROS_WARN("optimization not converged");
      return true;
    }
    for (int i = 0; i < n; i++) {
      points[i] =
          Eigen::Vector2d((var_new(i) + MAP_SIZE / 2) / PATH_CELL_SIZE,
                          (var_new(i + n) + MAP_SIZE / 2) / PATH_CELL_SIZE);
    }
    if (grad_new.norm() < 1e-6 * n) {
      return true;
    }
    return false;
  }
};
#endif  // SMOOTH_HPP