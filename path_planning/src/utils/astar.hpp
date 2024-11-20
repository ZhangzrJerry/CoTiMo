#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

namespace astar {
bool astar(const std::vector<std::vector<bool>>& grid_map,
           const Eigen::Vector2i& start, const Eigen::Vector2i& goal,
           std::vector<Eigen::Vector2i>& path) {
  int n = grid_map.size(), m = grid_map[0].size();
  std::vector<std::vector<int>> g(n, std::vector<int>(m, 0x3f3f3f3f));
  std::vector<std::vector<int>> f(n, std::vector<int>(m, 0x3f3f3f3f));
  std::vector<std::vector<Eigen::Vector2i>> parent(
      n, std::vector<Eigen::Vector2i>(m, Eigen::Vector2i(-1, -1)));

  std::vector<Eigen::Vector2i> dir = {
      Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, 0),  Eigen::Vector2i(0, -1),
      Eigen::Vector2i(-1, 0), Eigen::Vector2i(1, 1),  Eigen::Vector2i(1, -1),
      Eigen::Vector2i(-1, 1), Eigen::Vector2i(-1, -1)};

  std::multimap<int, Eigen::Vector2i> open;
  Eigen::Vector2i current;
  bool found = false;

  g[start(0)][start(1)] = 0;
  f[start(0)][start(1)] = (goal - start).lpNorm<2>();
  parent[start(0)][start(1)] = start;
  open.insert(std::make_pair(f[start(0)][start(1)], start));

  while (!open.empty()) {
    current = open.begin()->second;
    open.erase(open.begin());
    if (current == goal) {
      found = true;
      break;
    }
    for (auto& d : dir) {
      Eigen::Vector2i next = current + d;
      // out of bounds or obstacle
      if (next(0) < 0 || next(0) >= n || next(1) < 0 || next(1) >= m ||
          grid_map[next(0)][next(1)]) {
        continue;
      }
      if (d.norm() > 1 &&
          (grid_map[current(0)][next(1)] || grid_map[next(0)][current(1)])) {
        continue;
      }

      if (g[next(0)][next(1)] < 0x3f3f3f3f) {
        if (g[next(0)][next(1)] >
            g[current(0)][current(1)] + d.lpNorm<2>() +
                (parent[current(0)][current(1)] + next - 2 * current)
                    .lpNorm<1>()) {
          g[next(0)][next(1)] =
              g[current(0)][current(1)] + d.lpNorm<2>() +
              (parent[current(0)][current(1)] + next - 2 * current).lpNorm<1>();
          parent[next(0)][next(1)] = current;
          continue;
        }
      } else {
        g[next(0)][next(1)] =
            g[current(0)][current(1)] + d.lpNorm<2>() +
            (parent[current(0)][current(1)] + next - 2 * current).lpNorm<1>();
        parent[next(0)][next(1)] = current;
        open.insert(std::make_pair(
            g[next(0)][next(1)] + (goal - next).lpNorm<1>(), next));
      }
    }
  }
  path.clear();
  if (found) {
    while (current != start) {
      path.push_back(current);
      current = parent[current(0)][current(1)];
    }
    std::reverse(path.begin(), path.end());
    return true;
  }
  return false;
}
}  // namespace astar
#endif  // ASTAR_HPP