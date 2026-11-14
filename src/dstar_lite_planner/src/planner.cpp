// Nav2 global planner plugin around dstar_lite_planner::Core.
//
//   planner_server:
//     planner_plugins: [GridBased]
//     GridBased:
//       plugin: dstar_lite_planner::DStarLite
//       penalty_weight: 2.0      # extra cost of the most expensive free cell, as a multiple of a step
//       allow_unknown: false
//
// The search is kept between calls: while the goal stays the same only the cells that changed in
// the costmap are repaired and the robot's movement shifts the keys (km), which is what makes
// D* Lite cheaper than replanning from scratch when the costmap changes under a long path.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "dstar_lite_planner/core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#if __has_include("nav2_core/planner_exceptions.hpp")
#include "nav2_core/planner_exceptions.hpp"
namespace nav2_ex = nav2_core;
#define DSTAR_CANCEL_CHECKER 1
#else
// Humble: no planner exceptions and no cancel_checker in createPlan; plain exceptions instead.
#include <stdexcept>
namespace nav2_ex
{
struct Error : std::runtime_error {using std::runtime_error::runtime_error;};
using PlannerTFError = Error;
using StartOutsideMapBounds = Error;
using GoalOutsideMapBounds = Error;
using GoalOccupied = Error;
using StartOccupied = Error;
using PlannerCancelled = Error;
using NoValidPathCouldBeFound = Error;
}
#endif
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#if __has_include("tf2/utils.hpp")
#include "tf2/utils.hpp"
#else
#include "tf2/utils.h"
#endif

namespace dstar_lite_planner
{
class DStarLite : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  override
  {
    node_ = parent;
    auto node = parent.lock();
    name_ = name;
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".penalty_weight", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".allow_unknown", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".penalty_weight", penalty_weight_);
    node->get_parameter(name + ".allow_unknown", allow_unknown_);
    RCLCPP_INFO(logger_, "%s: D* Lite planner (penalty_weight %.2f, allow_unknown %s)",
      name.c_str(), penalty_weight_, allow_unknown_ ? "true" : "false");
  }

  void cleanup() override {core_.reset();}
  void activate() override {}
  void deactivate() override {}

#ifdef DSTAR_CANCEL_CHECKER
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {return plan(start, goal, cancel_checker);}
#else
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
  override
  {return plan(start, goal, nullptr);}
#endif

  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    const std::function<bool()> & cancel_checker)
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
      throw nav2_ex::PlannerTFError(
              "start/goal must be in the costmap frame '" + global_frame_ + "'");
    }
    unsigned int sx, sy, gx, gy;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
      throw nav2_ex::StartOutsideMapBounds("start is outside the costmap");
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
      throw nav2_ex::GoalOutsideMapBounds("goal is outside the costmap");
    }
    const unsigned int w = costmap_->getSizeInCellsX(), h = costmap_->getSizeInCellsY();
    const std::vector<double> cost = cost_grid(w, h, sx, sy);
    if (cost[gy * w + gx] < 0) {throw nav2_ex::GoalOccupied("goal cell is blocked");}
    if (cost[sy * w + sx] < 0) {throw nav2_ex::StartOccupied("start cell is in an obstacle");}
    if (cancel_checker && cancel_checker()) {throw nav2_ex::PlannerCancelled("cancelled");}

    const bool same_map = core_ && core_->width() == static_cast<int>(w) &&
      core_->height() == static_cast<int>(h) && res_ == costmap_->getResolution() &&
      ox_ == costmap_->getOriginX() && oy_ == costmap_->getOriginY();
    if (!same_map) {
      core_ = std::make_unique<Core>(w, h);
      res_ = costmap_->getResolution();
      ox_ = costmap_->getOriginX();
      oy_ = costmap_->getOriginY();
    }
    if (!core_->active() || core_->goal_x() != static_cast<int>(gx) ||
      core_->goal_y() != static_cast<int>(gy))
    {
      core_->reset(gx, gy, sx, sy);               // new goal / map: a fresh search
      core_->set_costs(cost);
    } else {
      core_->move_start(sx, sy);                  // same goal: repair only what changed
      core_->set_costs(cost);
    }
    if (!core_->compute()) {
      throw nav2_ex::NoValidPathCouldBeFound("D* Lite found no path to the goal");
    }
    RCLCPP_DEBUG(logger_, "D* Lite expanded %ld vertices", core_->expansions());

    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = clock_->now();
    const auto cells = core_->path();
    for (size_t i = 0; i < cells.size(); ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      double wx, wy;
      costmap_->mapToWorld(cells[i].first, cells[i].second, wx, wy);
      p.pose.position.x = wx;
      p.pose.position.y = wy;
      if (i == 0) {
        p.pose.position = start.pose.position;          // exact endpoints, cell centres between
      }
      if (i + 1 == cells.size()) {
        p.pose.position = goal.pose.position;
        p.pose.orientation = goal.pose.orientation;
      } else {
        double nx, ny;
        costmap_->mapToWorld(cells[i + 1].first, cells[i + 1].second, nx, ny);
        const double yaw = std::atan2(ny - wy, nx - wx);
        p.pose.orientation.z = std::sin(yaw / 2.0);
        p.pose.orientation.w = std::cos(yaw / 2.0);
      }
      path.poses.push_back(p);
    }
    return path;
  }

private:
  // costmap -> Core cost grid: free/inflated cells cost proportionally to their costmap value,
  // inscribed/lethal (and unknown, unless allowed) are blocked. The start cell is the exception:
  // a robot already inside the inflation zone must be able to leave it.
  std::vector<double> cost_grid(unsigned int w, unsigned int h, unsigned int sx, unsigned int sy)
  {
    std::vector<double> out(static_cast<size_t>(w) * h);
    for (unsigned int y = 0; y < h; ++y) {
      for (unsigned int x = 0; x < w; ++x) {
        const unsigned char c = costmap_->getCost(x, y);
        double v;
        if (c == nav2_costmap_2d::NO_INFORMATION) {
          v = allow_unknown_ ? penalty_weight_ * 0.5 : -1.0;
        } else if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          v = -1.0;
        } else {
          v = penalty_weight_ * static_cast<double>(c) / 252.0;
        }
        if (x == sx && y == sy && c != nav2_costmap_2d::LETHAL_OBSTACLE) {
          v = (v < 0) ? 0.0 : v;
        }
        out[static_cast<size_t>(y) * w + x] = v;
      }
    }
    return out;
  }

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("dstar_lite_planner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_, global_frame_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::unique_ptr<Core> core_;
  double res_ = 0, ox_ = 0, oy_ = 0;
  double penalty_weight_ = 2.0;
  bool allow_unknown_ = false;
};
}  // namespace dstar_lite_planner

PLUGINLIB_EXPORT_CLASS(dstar_lite_planner::DStarLite, nav2_core::GlobalPlanner)
