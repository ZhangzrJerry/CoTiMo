#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "config.h"
#include "map.hpp"
#include "mpc.hpp"
#include "obstacle.hpp"
#include "smooth.hpp"
#include "topp.hpp"
#include "utils/astar.hpp"
#include "utils/dijkstra.hpp"

/**
 * WAIT: wait for start
 * WAIT2: wait for goal
 * PATH: path finding
 * SMOOTH: optimize path
 * TOPP: run TOPP algorithm
 * SIM: simulation
 */
enum Mode { WAIT, WAIT2, PATH, SMOOTH, TOPP, SIM };
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
  ros::Rate rate(2000);

  // publishers & subscribers
  ros::Publisher map_pub = nh.advertise<nav_msgs::GridCells>("/path/map", 10);
  ros::Publisher path_pub =
      nh.advertise<visualization_msgs::Marker>("/path/path", 10);
  ros::Publisher curve_pub =
      nh.advertise<visualization_msgs::Marker>("/path/curve", 10);
  ros::Publisher grad_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/path/gradient", 10);
  ros::Publisher points_pub =
      nh.advertise<visualization_msgs::Marker>("/path/points", 10);
  ros::Publisher sim_pub =
      nh.advertise<visualization_msgs::Marker>("/path/sim", 10);
  ros::Publisher plot_at_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/at", 10);
  ros::Publisher plot_ax_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/ax", 10);
  ros::Publisher plot_ay_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/ay", 10);
  ros::Publisher plot_vt_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/vt", 10);
  ros::Publisher plot_vx_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/vx", 10);
  ros::Publisher plot_vy_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/vy", 10);
  ros::Subscriber goal_sub = nh.subscribe("/rviz/goal", 10, goalCallback);

  // initialize map
  nav_msgs::GridCells map_msg;
  Map* map = Map::getInstance();
  map->getMap(map_msg);
  ROS_INFO("map initialized");

  std::vector<Eigen::Vector2d> points, acceleration, velocity;
  Eigen::Vector2d xx, vv, aa;
  std::vector<double> dt;
  Smooth* smooth;
  Topp* topp;
  Mpc* mpc;
  visualization_msgs::Marker sim_msg;
  std_msgs::Float64MultiArray plot_at_msg, plot_ax_msg, plot_ay_msg,
      plot_vt_msg, plot_vx_msg, plot_vy_msg;
  int sim_idx = 0;
  double sim_t = 0;

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
        //   ROS_WARN("no path found");
        //   mode = Mode::WAIT;
        //   break;
        // }
        if (!astar::astar(grid_map, start, goal, path)) {
          ROS_WARN("no path found");
          mode = Mode::WAIT;
          break;
        }
        ROS_INFO("path found, length: %ld", path.size());

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
        ROS_INFO("control points: %ld", points.size());

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

        // visualize control points
        visualization_msgs::Marker points_msg;
        map->getPoints(points_msg, points);
        points_pub.publish(points_msg);

        if (flag) {
          ROS_INFO("path optimized, start TOPP");
          mode = Mode::TOPP;
          grad = Eigen::VectorXd::Zero(2 * points.size());
          map->getGradient(grad_msg, points, grad);
          grad_pub.publish(grad_msg);
          topp = new Topp(points);
        }
        break;
      }
      case Mode::TOPP: {
        bool flag = topp->step();
        if (flag) {
          ROS_INFO("TOPP finished, get params");
          mode = Mode::SIM;
          topp->get(dt, velocity, acceleration);
          plot_at_msg.data.clear();
          plot_ax_msg.data.clear();
          plot_ay_msg.data.clear();
          plot_vt_msg.data.clear();
          plot_vx_msg.data.clear();
          plot_vy_msg.data.clear();
          xx = points[0];
          sim_idx = 0;
          sim_t = 0;
          ROS_INFO("params obtained, start to simulate");
        }
        break;
      }
      case Mode::SIM: {
        // I am too lazy to add some noise and implement an optimal controller
        // because of the 5 coming deadlines in this week
        // Make a wish. If there are more than 50 stars, I will add this
        // function and reorganize the messy code after the deadlines.

        xx = points[sim_idx] + sim_t * (points[sim_idx + 1] - points[sim_idx]) /
                                   (1e-16 + dt[sim_idx + 1] - dt[sim_idx]);
        plot_at_msg.data.push_back(dt[sim_idx] + sim_t);
        plot_ax_msg.data.push_back(acceleration[sim_idx](0));
        plot_ay_msg.data.push_back(acceleration[sim_idx](1));
        if (sim_t >= dt[sim_idx + 1] - dt[sim_idx]) {
          plot_vt_msg.data.push_back(dt[sim_idx]);
          plot_vx_msg.data.push_back(velocity[sim_idx](0));
          plot_vy_msg.data.push_back(velocity[sim_idx](1));
          ++sim_idx;
          sim_t = 0;
        }
        sim_t += DT;

        visualization_msgs::Marker sim_msg;
        map->getPoint(sim_msg, xx);
        sim_pub.publish(sim_msg);

        if (sim_idx >= dt.size() - 1) {
          ROS_INFO("simulation finished");
          mode = Mode::WAIT;
          plot_at_pub.publish(plot_at_msg);
          plot_ax_pub.publish(plot_ax_msg);
          plot_ay_pub.publish(plot_ay_msg);
          plot_vt_pub.publish(plot_vt_msg);
          plot_vx_pub.publish(plot_vx_msg);
          plot_vy_pub.publish(plot_vy_msg);
          visualization_msgs::Marker sim_msg;
          sim_msg.header.frame_id = "map";
          sim_msg.ns = "sim";
          sim_msg.id = 0;
          sim_msg.type = visualization_msgs::Marker::SPHERE;
          sim_msg.action = visualization_msgs::Marker::DELETE;
          sim_msg.color.a = 0;
          sim_pub.publish(sim_msg);
          break;
        }
        break;
      }
      default: {
        mode = Mode::WAIT;
        break;
      }
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}