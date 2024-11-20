#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "config.h"
#include "map.hpp"
#include "obstacle.hpp"
#include "smooth.hpp"
#include "utils/astar.hpp"
#include "utils/dijkstra.hpp"

/**
 * WAIT: wait for start
 * WAIT2: wait for goal
 * PATH: path finding
 * SMOOTH: optimize path
 * TOPP: run TOPP algorithm
 */
enum Mode { WAIT, WAIT2, PATH, SMOOTH, TOPP };
Eigen::Vector2i start, current, goal;
Mode mode = Mode::WAIT;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // set goal
  if (mode == Mode::WAIT2) {
    goal(0) = std::min(std::max(msg->pose.position.x / PATH_CELL_SIZE, 0.0),
                       MAP_SIZE / PATH_CELL_SIZE);
    goal(1) = std::min(std::max(msg->pose.position.y / PATH_CELL_SIZE, 0.0),
                       MAP_SIZE / PATH_CELL_SIZE);
    mode = Mode::PATH;
    ROS_INFO("goal set to (%.2f, %.2f)", msg->pose.position.x - MAP_SIZE / 2,
             msg->pose.position.y - MAP_SIZE / 2);
    return;
  }
  // set start
  start(0) = std::min(std::max(msg->pose.position.x / PATH_CELL_SIZE, 0.0),
                      MAP_SIZE / PATH_CELL_SIZE);
  start(1) = std::min(std::max(msg->pose.position.y / PATH_CELL_SIZE, 0.0),
                      MAP_SIZE / PATH_CELL_SIZE);
  current = start;
  mode = Mode::WAIT2;
  ROS_INFO("start set to (%.2f, %.2f)", msg->pose.position.x - MAP_SIZE / 2,
           msg->pose.position.y - MAP_SIZE / 2);
  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_node");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  // publishers & subscribers
  ros::Publisher map_pub = nh.advertise<nav_msgs::GridCells>("/path/map", 10);
  ros::Publisher path_pub =
      nh.advertise<visualization_msgs::Marker>("/path/path", 10);
  ros::Publisher curve_pub =
      nh.advertise<visualization_msgs::Marker>("/path/curve", 10);
  ros::Publisher grad_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/path/gradient", 10);
  ros::Subscriber goal_sub = nh.subscribe("/rviz/goal", 10, goalCallback);

  // initialize map
  nav_msgs::GridCells map_msg;
  Map* map = Map::getInstance();
  map->getMap(map_msg);
  ROS_INFO("map initialized");

  std::vector<Eigen::Vector2d> points;
  Smooth* smooth;

  while (ros::ok()) {
    switch (mode) {
      case Mode::WAIT: {
        // visualize map
        map_pub.publish(map_msg);
        break;
      }
      case Mode::WAIT2: {
        break;
      }
      case Mode::PATH: {
        // find path
        std::vector<std::vector<bool>> grid_map(
            MAP_SIZE / PATH_CELL_SIZE,
            std::vector<bool>(MAP_SIZE / PATH_CELL_SIZE));
        map->getGridMap(grid_map);
        std::vector<Eigen::Vector2i> path;
        // if (!dijkstra::dijkstra(grid_map, start, goal, path)) {
        //   mode = Mode::WAIT;
        //   break;
        // }
        if (!astar::astar(grid_map, start, goal, path)) {
          mode = Mode::WAIT;
          break;
        }

        // visualize path
        visualization_msgs::Marker path_msg;
        map->getPath(path_msg, path);
        path_pub.publish(path_msg);

        // select control points
        points.clear();
        points.push_back(start.cast<double>());
        for (int i = 1; i < path.size() - 1; i += INTERVAL) {
          points.push_back(path[i].cast<double>());
        }
        points.push_back(goal.cast<double>());
        if (points.size() < 3) {
          ROS_WARN("too few control points");
          mode = Mode::WAIT;
          break;
        }
        ROS_INFO("control points: %d", points.size());

        smooth = new Smooth(points.size());
        mode = Mode::SMOOTH;
        break;
      }
      case Mode::SMOOTH: {
        Eigen::VectorXd grad;
        bool flag = smooth->step(points, grad);
        // visualize optimized path
        visualization_msgs::Marker curve_msg;
        map->getCurve(curve_msg, points);
        curve_pub.publish(curve_msg);

        // visualize gradient
        visualization_msgs::MarkerArray grad_msg;
        map->getGradient(grad_msg, points, grad);
        grad_pub.publish(grad_msg);
        if (flag) {
          ROS_INFO("path optimized, start TOPP");
          mode = Mode::TOPP;
        }
        break;
      }
      case Mode::TOPP: {
        break;
      }
      default: {
        break;
      }
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}