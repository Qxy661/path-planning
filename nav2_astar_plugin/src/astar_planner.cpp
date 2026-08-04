// Copyright 2026 Qxy661
//
// Hand-written A* as a Nav2 global planner plugin.

#include "astar_nav2_plugin/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace astar_nav2_plugin
{

void AstarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();
  logger_ = rclcpp::get_logger(name_);

  // 可选参数：障碍阈值
  auto node = parent.lock();
  if (node) {
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".lethal_threshold", rclcpp::ParameterValue(200));
    node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
    RCLCPP_INFO(logger_, "A* planner configured: res=%.3f origin=(%.2f,%.2f) threshold=%d",
      resolution_, origin_x_, origin_y_, lethal_threshold_);
  }
}

void AstarPlanner::activate() {}
void AstarPlanner::deactivate() {}
void AstarPlanner::cleanup()
{
  costmap_ = nullptr;
  costmap_ros_.reset();
}

std::vector<unsigned int> AstarPlanner::a_star_search(
  unsigned int start_index, unsigned int goal_index)
{
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const unsigned int size = size_x * size_y;

  // 8 邻域
  const int dx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  const int dy[8] = {1, 1, 0, -1, -1, -1, 0, 1};

  std::vector<double> g_cost(size, std::numeric_limits<double>::infinity());
  std::vector<unsigned int> came_from(size, size);  // size = 哨兵（无父节点）
  std::vector<bool> closed(size, false);

  auto to_index = [&](unsigned int x, unsigned int y) { return y * size_x + x; };

  const unsigned int sx = start_index % size_x;
  const unsigned int sy = start_index / size_x;
  const unsigned int gx = goal_index % size_x;
  const unsigned int gy = goal_index / size_x;

  // 启发式：欧氏距离（8 邻域）
  auto heuristic = [&](unsigned int x, unsigned int y) {
    const double dx_ = static_cast<int>(gx) - static_cast<int>(x);
    const double dy_ = static_cast<int>(gy) - static_cast<int>(y);
    return std::sqrt(dx_ * dx_ + dy_ * dy_);
  };

  using Node = std::pair<double, unsigned int>;  // (f = g + h, index)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

  g_cost[start_index] = 0.0;
  open.emplace(heuristic(sx, sy), start_index);

  while (!open.empty()) {
    const auto [f, idx] = open.top();
    open.pop();
    if (closed[idx]) { continue; }
    closed[idx] = true;
    if (idx == goal_index) { break; }

    const unsigned int cx = idx % size_x;
    const unsigned int cy = idx / size_x;
    for (int k = 0; k < 8; ++k) {
      const int nx = static_cast<int>(cx) + dx[k];
      const int ny = static_cast<int>(cy) + dy[k];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y)) {
        continue;
      }
      const unsigned int nidx = to_index(nx, ny);
      if (closed[nidx]) { continue; }
      if (costmap_->getCost(nx, ny) > lethal_threshold_) { continue; }

      const double move_cost = (dx[k] != 0 && dy[k] != 0) ? 1.41421356 : 1.0;
      const double ng = g_cost[idx] + move_cost;
      if (ng < g_cost[nidx]) {
        g_cost[nidx] = ng;
        came_from[nidx] = idx;
        open.emplace(ng + heuristic(nx, ny), nidx);
      }
    }
  }

  std::vector<unsigned int> path;
  if (closed[goal_index] || start_index == goal_index) {
    unsigned int cur = goal_index;
    while (cur != size) {
      path.push_back(cur);
      cur = came_from[cur];
    }
    std::reverse(path.begin(), path.end());
  }
  return path;
}

nav_msgs::msg::Path AstarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header = goal.header;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  // 关键：createPlan 时重新读取 costmap 的 origin/resolution
  // （configure 时地图可能尚未加载，origin 为 (0,0) 默认值）
  const double res = costmap_->getResolution();
  const double ox = costmap_->getOriginX();
  const double oy = costmap_->getOriginY();

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();

  const auto world_to_index = [&](double wx, double wy, unsigned int * xi, unsigned int * yi) -> bool {
    const int ix = static_cast<int>((wx - ox) / res);
    const int iy = static_cast<int>((wy - oy) / res);
    if (ix < 0 || iy < 0 || ix >= static_cast<int>(size_x) || iy >= static_cast<int>(size_y)) {
      return false;
    }
    *xi = ix; *yi = iy;
    return true;
  };

  unsigned int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!world_to_index(start.pose.position.x, start.pose.position.y, &sx, &sy) ||
      !world_to_index(goal.pose.position.x, goal.pose.position.y, &gx, &gy))
  {
    RCLCPP_WARN(logger_, "Start or goal out of costmap bounds");
    return path;
  }

  // 若起点/终点落在障碍上，回退到最近空闲格
  const auto nearest_free = [&](unsigned int & x, unsigned int & y) {
    if (costmap_->getCost(x, y) <= lethal_threshold_) { return; }
    for (int d = 1; d < 30; ++d) {
      for (int dr = -d; dr <= d; ++dr) {
        for (int dc = -d; dc <= d; ++dc) {
          int nx = static_cast<int>(x) + dr;
          int ny = static_cast<int>(y) + dc;
          if (nx >= 0 && ny >= 0 && nx < static_cast<int>(size_x) && ny < static_cast<int>(size_y)) {
            if (costmap_->getCost(nx, ny) <= lethal_threshold_) {
              x = nx; y = ny; return;
            }
          }
        }
      }
    }
  };
  nearest_free(sx, sy);
  nearest_free(gx, gy);

  RCLCPP_INFO(logger_, "A* start grid=(%u,%u) goal grid=(%u,%u) costmap=%ux%u origin=(%.2f,%.2f) res=%.3f",
    sx, sy, gx, gy, size_x, size_y, ox, oy, res);

  const auto idx = [&](unsigned int x, unsigned int y) { return y * size_x + x; };
  std::vector<unsigned int> grid_path = a_star_search(idx(sx, sy), idx(gx, gy));

  if (grid_path.empty()) {
    RCLCPP_WARN(logger_, "A* failed to find a path from (%.2f,%.2f) to (%.2f,%.2f)",
      start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    return path;
  }

  // 栅格 → 世界坐标
  path.poses.reserve(grid_path.size());
  for (const unsigned int i : grid_path) {
    const unsigned int cx = i % size_x;
    const unsigned int cy = i / size_x;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = ox + (cx + 0.5) * res;
    pose.pose.position.y = oy + (cy + 0.5) * res;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  RCLCPP_INFO(logger_, "A* plan: %zu poses, %.2f m", path.poses.size(),
    grid_path.size() * res);
  return path;
}

}  // namespace astar_nav2_plugin

PLUGINLIB_EXPORT_CLASS(astar_nav2_plugin::AstarPlanner, nav2_core::GlobalPlanner)
