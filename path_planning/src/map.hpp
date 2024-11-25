#ifndef MAP_HPP
#define MAP_HPP

#include <nav_msgs/GridCells.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "config.h"
#include "obstacle.hpp"
#include "utils/spline.hpp"

class Map {
 private:
  static Map* instance;  // Declaration of the static member
  std::vector<Obstacle<2>> obstacles;

  Map() {
    obstacles.clear();

    // map boundaries
    Eigen::Matrix<double, 1, 2> A;
    Eigen::Matrix<double, 1, 1> b;
    A(0, 0) = 1, A(0, 1) = 0, b(0) = -10;
    obstacles.push_back(Obstacle<2>(1, A, b));
    A(0, 0) = 0, A(0, 1) = 1, b(0) = -10;
    obstacles.push_back(Obstacle<2>(1, A, b));
    A(0, 0) = -1, A(0, 1) = 0, b(0) = -10;
    obstacles.push_back(Obstacle<2>(1, A, b));
    A(0, 0) = 0, A(0, 1) = -1, b(0) = -10;
    obstacles.push_back(Obstacle<2>(1, A, b));

    /**
     * -0.5 * x + 1 * y <=  12
     *   -1 * x + 0 * y <=   8
     *  0.5 * x - 1 * y <= -10
     *    1 * x - 1 * y <= -12
     *    0 * x + 1 * y <=  10
     */
    Eigen::Matrix<double, 5, 2> A1;
    Eigen::Matrix<double, 5, 1> b1;
    A1 << -0.5, 1, -1, 0, 0.5, -1, 1, -1, 0, 1;
    b1 << 12, 8, -10, -12, 10;
    obstacles.push_back(Obstacle<2>(5, A1, b1));

    /**
     * 1 * x + 1 * y <=  12.5
     * 1 * x - 1 * y <=  4
     * 0 * x - 1 * y <=  -2.5
     * -1 * x + 0 * y <= -4
     * -1 * x + 1 * y <= -3.5
     * 0 * x + 1 * y <=  2
     * 1 * x + 0 * y <=  9.5
     * 1 * x + 1 * y <=  10
     */
    Eigen::Matrix<double, 8, 2> A2;
    Eigen::Matrix<double, 8, 1> b2;
    A2 << 1, 1, 1, 0, 1, -1, 0, -1, -1, -1, -1, 0, -1, 1, 0, 1;
    b2 << 12.5, 4, -2.5, -4, -3.5, 2, 9.5, 10;
    obstacles.push_back(Obstacle<2>(8, A2, b2));

    /**
     * -1/3 * x + 1 * y <=  22/3
     * 1 * x + 1 * y <=  2
     * 0.25 * x - 1 * y <=  -4.5
     * -1 * x + 0 * y <= 10
     */
    Eigen::Matrix<double, 4, 2> A3;
    Eigen::Matrix<double, 4, 1> b3;
    A3 << -1.0 / 3, 1, 1, 1, 0.25, -1, -1, 0;
    b3 << 22.0 / 3, 2, -4.5, 10;
    obstacles.push_back(Obstacle<2>(4, A3, b3));

    /**
     * 0 * x + 1 * y <=  10
     * 1 * x + 0 * y <=  10
     * 0 * x - 1 * y <=  -8
     * -1 * x -1 * y<=-14
     */
    Eigen::Matrix<double, 4, 2> A4;
    Eigen::Matrix<double, 4, 1> b4;
    A4 << 0, 1, 1, 0, 0, -1, -1, -1;
    b4 << 10, 10, -8, -14;
    obstacles.push_back(Obstacle<2>(4, A4, b4));

    /**
     * 1 * x + 0 * y <=  10
     * 0 * x + 1 * y <=  6
     * 0 * x - 1 * y <=  0
     * -1 * x + 1 * y <=  0
     * -1 * x - 1 * y <= -8
     */
    Eigen::Matrix<double, 5, 2> A5;
    Eigen::Matrix<double, 5, 1> b5;
    A5 << 1, 0, 0, 1, 0, -1, -1, 1, -1, -1;
    b5 << 10, 6, 0, 0, -8;
    obstacles.push_back(Obstacle<2>(5, A5, b5));

    /**
     * -x-y<=10
     * -x+2y<=10
     * 2x-y<=-14
     */
    Eigen::Matrix<double, 3, 2> A6;
    Eigen::Matrix<double, 3, 1> b6;
    A6 << -1, -1, -1, 2, 2, -1;
    b6 << 10, 10, -14;
    obstacles.push_back(Obstacle<2>(3, A6, b6));

    /**
     * - x + y <=4
     * x + 0 * y <= -2
     * -2x-y<=14
     * x-y<=2
     */
    Eigen::Matrix<double, 4, 2> A7;
    Eigen::Matrix<double, 4, 1> b7;
    A7 << -1, 1, 1, 0, -2, -1, 1, -1;
    b7 << 4, -2, 14, 2;
    obstacles.push_back(Obstacle<2>(4, A7, b7));

    /**
     * x <= -4
     * -y <= 10
     * -3x + y <= 20
     * x + y <= -12
     */
    Eigen::Matrix<double, 4, 2> A8;
    Eigen::Matrix<double, 4, 1> b8;
    A8 << 1, 0, 0, -1, -3, 1, 1, 1;
    b8 << -4, 10, 20, -12;
    obstacles.push_back(Obstacle<2>(4, A8, b8));

    /**
     * x + y <= -4
     * x <= 2
     * -x <= 2
     * -2x+y<=-4
     */
    Eigen::Matrix<double, 4, 2> A9;
    Eigen::Matrix<double, 4, 1> b9;
    A9 << 1, 1, 1, 0, -1, 0, -2, 1;
    b9 << -4, 2, 2, -4;
    obstacles.push_back(Obstacle<2>(4, A9, b9));

    /**
     * x+2y<=6
     * -x-2y<=4
     * x-y<=8
     * -x<=0
     * x<=6
     * y<=2
     */
    Eigen::Matrix<double, 6, 2> A10;
    Eigen::Matrix<double, 6, 1> b10;
    A10 << 1, 2, -1, -2, 1, -1, -1, 0, 1, 0, 0, 1;
    b10 << 6, 4, 8, 0, 6, 2;
    obstacles.push_back(Obstacle<2>(6, A10, b10));

    /**
     * -y <= 10
     * -x <= -4
     * -x + y <= -12
     * 2x + y <= 6
     */
    Eigen::Matrix<double, 4, 2> A11;
    Eigen::Matrix<double, 4, 1> b11;
    A11 << 0, -1, -1, 0, -1, 1, 2, 1;
    b11 << 10, -4, -12, 6;
    obstacles.push_back(Obstacle<2>(4, A11, b11));

    /**
     * -x - y <= -2
     * -x+y<=-10
     * x<=10
     * y<=-2
     */
    Eigen::Matrix<double, 4, 2> A12;
    Eigen::Matrix<double, 4, 1> b12;
    A12 << -1, -1, -1, 1, 1, 0, 0, 1;
    b12 << -2, -10, 10, -2;
    obstacles.push_back(Obstacle<2>(4, A12, b12));
  }
  Map(const Map&) = delete;

 public:
  static Map* getInstance() {
    if (instance == nullptr) {
      instance = new Map();
    }
    return instance;
  }

  inline bool occupied(const Eigen::Vector2d& x) {
    for (auto& obstacle : obstacles) {
      if (obstacle.inside(x)) {
        return true;
      }
    }
    return false;
  }

  inline double distance(Eigen::Vector2d x) {
    double res = -0x3f3f3f3f;
    for (auto& obstacle : obstacles) {
      res = std::max(res, obstacle.distance(x));
    }
    return res;
  }

  inline void getMap(nav_msgs::GridCells& map_msg) {
    map_msg.header.frame_id = "map";
    map_msg.cell_width = CELL_SIZE;
    map_msg.cell_height = CELL_SIZE;
    for (int i = 0; i < MAP_SIZE / CELL_SIZE; ++i) {
      for (int j = 0; j < MAP_SIZE / CELL_SIZE; ++j) {
        Eigen::Vector2d x((i + .5) * CELL_SIZE - MAP_SIZE / 2,
                          (j + .5) * CELL_SIZE - MAP_SIZE / 2);
        if (occupied(x)) {
          geometry_msgs::Point p;
          p.x = (i + .5) * CELL_SIZE;
          p.y = (j + .5) * CELL_SIZE;
          p.z = 0;
          map_msg.cells.push_back(p);
        }
      }
    }
    return;
  }

  inline void getPath(visualization_msgs::Marker& path_msg,
                      const std::vector<Eigen::Vector2i>& path) {
    path_msg.header.frame_id = "map";
    path_msg.ns = "path";
    path_msg.id = 0;
    path_msg.type = visualization_msgs::Marker::LINE_STRIP;
    path_msg.action = visualization_msgs::Marker::ADD;
    path_msg.scale.x = RENDER_SIZE;
    path_msg.pose.orientation.w = 1.0;
    path_msg.color.a = .6;
    path_msg.color.r = 1;
    path_msg.color.g = 0;
    path_msg.color.b = 0;

    geometry_msgs::Point p;
    for (auto& x : path) {
      p.x = (x(0) + .5) * RENDER_SIZE;
      p.y = (x(1) + .5) * RENDER_SIZE;
      p.z = 0;
      path_msg.points.push_back(p);
    }
    return;
  }

  inline void getCurve(visualization_msgs::Marker& curve_msg,
                       const std::vector<Eigen::Vector2d>& points) {
    curve_msg.header.frame_id = "map";
    curve_msg.ns = "curve";
    curve_msg.id = 0;
    curve_msg.type = visualization_msgs::Marker::LINE_STRIP;
    curve_msg.action = visualization_msgs::Marker::ADD;
    curve_msg.scale.x = RENDER_SIZE;
    curve_msg.pose.orientation.w = 1.0;
    curve_msg.color.a = .9;
    curve_msg.color.r = 0;
    curve_msg.color.g = 0;
    curve_msg.color.b = 1;

    geometry_msgs::Point p;
    if (points.size() < 3) {
      for (int i = 0; i < points.size(); ++i) {
        p.x = (points[i](0) + .5) * RENDER_SIZE;
        p.y = (points[i](1) + .5) * RENDER_SIZE;
        p.z = 0;
        curve_msg.points.push_back(p);
      }
      return;
    }

    int n = points.size();
    Eigen::VectorXd x(n), y(n);
    for (int i = 0; i < n; ++i) {
      x(i) = points[i](0), y(i) = points[i](1);
    }
    Eigen::VectorXd ax, bx, cx, dx, ay, by, cy, dy;
    spline::spline(x, ax, bx, cx, dx);
    spline::spline(y, ay, by, cy, dy);

    for (int i = 0; i < n - 1; ++i) {
      for (double t = 0; t < 1; t += .01) {
        double px = ax(i) * t * t * t + bx(i) * t * t + cx(i) * t + dx(i),
               py = ay(i) * t * t * t + by(i) * t * t + cy(i) * t + dy(i);
        p.x = (px + .5) * RENDER_SIZE;
        p.y = (py + .5) * RENDER_SIZE;
        p.z = 0;
        curve_msg.points.push_back(p);
      }
    }
    return;
  }

  inline void getPoints(visualization_msgs::Marker& points_msg,
                        const std::vector<Eigen::Vector2d>& points) {
    points_msg.header.frame_id = "map";
    points_msg.ns = "points";
    points_msg.id = 0;
    points_msg.type = visualization_msgs::Marker::SPHERE_LIST;
    points_msg.action = visualization_msgs::Marker::ADD;
    points_msg.scale.x = RENDER_SIZE * 2;
    points_msg.scale.y = RENDER_SIZE * 2;
    points_msg.scale.z = RENDER_SIZE * 2;
    points_msg.pose.orientation.w = 1.0;
    points_msg.color.a = .9;
    points_msg.color.r = 1.;
    points_msg.color.g = 1.;
    points_msg.color.b = 0;

    geometry_msgs::Point p;
    for (int i = 0; i < points.size(); ++i) {
      p.x = (points[i](0) + .5) * RENDER_SIZE;
      p.y = (points[i](1) + .5) * RENDER_SIZE;
      p.z = .01;
      points_msg.points.push_back(p);
    }
    return;
  }

  inline void getPoint(visualization_msgs::Marker& point_msg,
                       const Eigen::Vector2d& point) {
    point_msg.header.frame_id = "map";
    point_msg.ns = "point";
    point_msg.id = 0;
    point_msg.type = visualization_msgs::Marker::SPHERE;
    point_msg.scale.x = RENDER_SIZE * 4;
    point_msg.scale.y = RENDER_SIZE * 4;
    point_msg.scale.z = RENDER_SIZE * 4;
    point_msg.pose.orientation.w = 1.0;
    point_msg.color.a = .9;
    point_msg.color.r = 1.;
    point_msg.color.g = 1.;
    point_msg.color.b = 0;

    geometry_msgs::Point p;
    p.x = (point(0) + .5) * RENDER_SIZE;
    p.y = (point(1) + .5) * RENDER_SIZE;
    p.z = .05;
    point_msg.pose.position = p;
    return;
  }

  inline void getGradient(visualization_msgs::MarkerArray& grad_msg,
                          const std::vector<Eigen::Vector2d>& points,
                          const Eigen::VectorXd& grad) {
    for (int i = 0; i < grad_msg.markers.size(); ++i) {
      grad_msg.markers[i].action = visualization_msgs::Marker::DELETE;
      grad_msg.markers[i].color.a = 0;
    }
    grad_msg.markers.clear();
    for (int i = 0; i < points.size(); ++i) {
      if (Eigen::Vector2d(grad(i), grad(i + points.size())).norm() < 1e-3) {
        continue;
      }
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "grad";
      marker.lifetime = ros::Duration();
      marker.id = i;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = RENDER_SIZE / 2;
      marker.scale.y = RENDER_SIZE * 1.2;
      marker.scale.z = RENDER_SIZE / 2;
      marker.pose.orientation.w = 1.0;
      marker.color.a = .9;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;

      geometry_msgs::Point p;
      p.x = (points[i](0) + .5) * RENDER_SIZE;
      p.y = (points[i](1) + .5) * RENDER_SIZE;
      p.z = 0;
      marker.points.push_back(p);
      p.x += grad(i) * RENDER_SIZE;
      p.y += grad(i + points.size()) * RENDER_SIZE;
      marker.points.push_back(p);
      grad_msg.markers.push_back(marker);
    }
    return;
  }

  inline void getGridMap(std::vector<std::vector<bool>>& grid_map) {
    if (grid_map.size() != MAP_SIZE / PATH_CELL_SIZE ||
        grid_map[0].size() != MAP_SIZE / PATH_CELL_SIZE) {
      ROS_ERROR("grid_map size mismatch");
      return;
    }
    for (int i = 0; i < MAP_SIZE / PATH_CELL_SIZE; ++i) {
      for (int j = 0; j < MAP_SIZE / PATH_CELL_SIZE; ++j) {
        grid_map[i][j] =
            occupied(Eigen::Vector2d((i + .5) * PATH_CELL_SIZE - MAP_SIZE / 2,
                                     (j + .5) * PATH_CELL_SIZE - MAP_SIZE / 2));
      }
    }
    return;
  }
};

// Definition of the static member
Map* Map::instance = nullptr;

#endif  // MAP_HPP