// $ /ws/devel/lib/path_planning/test_demo

#include "eigen3/Eigen/Eigen"
#include "map.hpp"
#include "obstacle.hpp"

int main(int argc, char** argv) {
  // SDQP test
  Eigen::Matrix<double, 5, 2> A;
  Eigen::Matrix<double, 5, 1> b;
  A << -0.5, 1, -1, 0, 0.5, -1, 1, -1, 0, 1;
  b << 12, 8, -10, -12, 10;
  Obstacle<2> obstacle(5, A, b);

  Eigen::Vector2d p1(-10, 10);  // 2.82843
  Eigen::Vector2d p2(-8, 10);   // 1.78885
  Eigen::Vector2d p3(-6, 10);   // 0.89443
  Eigen::Vector2d p4(-10, 8);   // 2
  Eigen::Vector2d p5(-6, 9);    // 0
  Eigen::Vector2d p6(-6, 8);

  std::cout << "p1: " << obstacle.inside(p1) << ", " << obstacle.distance(p1)
            << std::endl;
  std::cout << "p2: " << obstacle.inside(p2) << ", " << obstacle.distance(p2)
            << std::endl;
  std::cout << "p3: " << obstacle.inside(p3) << ", " << obstacle.distance(p3)
            << std::endl;
  std::cout << "p4: " << obstacle.inside(p4) << ", " << obstacle.distance(p4)
            << std::endl;
  std::cout << "p5: " << obstacle.inside(p5) << ", " << obstacle.distance(p5)
            << std::endl;
  std::cout << "p6: " << obstacle.inside(p6) << ", " << obstacle.distance(p6)
            << std::endl;

  std::cout << "p7: "
            << Map::getInstance()->occupied(Eigen::Vector2d(-10.001, 10.001))
            << ", "
            << Map::getInstance()->distance(Eigen::Vector2d(-10.001, 10.001))
            << std::endl;
  std::cout << "p8: "
            << Map::getInstance()->occupied(Eigen::Vector2d(-9.9999, 9.9999))
            << ", "
            << Map::getInstance()->distance(Eigen::Vector2d(-9.9999, 9.9999))
            << std::endl;

  return 0;
}