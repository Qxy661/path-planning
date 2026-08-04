// Copyright 2026 Qxy661
//
// Hand-written A* as a Nav2 global planner plugin.
// Demonstrates: 手写算法 → Nav2 插件化集成（研究验证的最强作品集模式）
//
// Reference API: https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html

#ifndef ASTAR_NAV2_PLUGIN__ASTAR_PLANNER_HPP_
#define ASTAR_NAV2_PLUGIN__ASTAR_PLANNER_HPP_

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace astar_nav2_plugin
{

/**
 * A* 全局规划器（手写实现）
 *
 * 把代价地图栅格化为 0/1 二值图 → 手写 A* 图搜索 → 输出全局路径。
 * 通过 pluginlib 导出，在 nav2_params.yaml 配置即可切换使用。
 */
class AstarPlanner : public nav2_core::GlobalPlanner
{
public:
  AstarPlanner() = default;
  ~AstarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /** A* 核心搜索：返回从 start_index 到 goal_index 的栅格下标路径（含两端）*/
  std::vector<unsigned int> a_star_search(unsigned int start_index, unsigned int goal_index);

  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("astar_planner")};

  double resolution_;
  double origin_x_;
  double origin_y_;
  // 障碍阈值：代价 > 该值视为不可通行（默认 200，避让膨胀区）
  unsigned char lethal_threshold_{200};
};

}  // namespace astar_nav2_plugin

#endif  // ASTAR_NAV2_PLUGIN__ASTAR_PLANNER_HPP_
