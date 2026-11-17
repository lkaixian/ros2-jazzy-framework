#pragma once
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "bt_pipeline_msgs/action/execute_node.hpp"
#include "bt_pipeline_msgs/msg/extension_manifest.hpp"
#include "face_recognition_msgr/msg/face_array.hpp"
#include "object_detection_msgr/msg/object_array.hpp"
#include "person_detection_msgr/msg/person_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace bt_executor
{
using Clock = std::chrono::steady_clock;
using ExecuteNode = bt_pipeline_msgs::action::ExecuteNode;

/// Latest perception, queried by the built-in condition nodes.
class PerceptionCache
{
public:
  void on_objects(const object_detection_msgr::msg::ObjectArray & m, Clock::time_point now);
  void on_persons(const person_detection_msgr::msg::PersonArray & m, Clock::time_point now);
  void on_faces(const face_recognition_msgr::msg::FaceArray & m, Clock::time_point now);

  /// bbox_out: "x_min,y_min,x_max,y_max"
  bool object(const std::string & label, double min_conf, double max_age_sec,
    Clock::time_point now, std::string * bbox_out) const;
  bool person(double min_conf, double max_age_sec, Clock::time_point now,
    int * track_out) const;
  bool face(const std::string & identity, double min_similarity, double max_age_sec,
    Clock::time_point now) const;
  /// Largest face that has an embedding (a feature was extracted). Outputs may be null.
  bool face_feature(double min_conf, double max_age_sec, Clock::time_point now,
    std::string * identity, double * similarity) const;

private:
  mutable std::mutex mu_;
  object_detection_msgr::msg::ObjectArray objects_;
  person_detection_msgr::msg::PersonArray persons_;
  face_recognition_msgr::msg::FaceArray faces_;
  Clock::time_point t_obj_{}, t_per_{}, t_face_{};
  bool has_obj_ = false, has_per_ = false, has_face_ = false;
};

/// Shared by every extension node of one run.
struct ExecContext
{
  rclcpp::Node::SharedPtr node;   // may be null when dry_run (tests)
  bool dry_run = true;
  std::set<std::string> dry_run_exempt;   // extensions that run for real even in a dry run (speech: no motion)
  std::function<void(const std::string &)> log = [](const std::string &) {};
  rclcpp_action::Client<ExecuteNode>::SharedPtr client(const std::string & extension);

private:
  std::mutex mu_;
  std::map<std::string, rclcpp_action::Client<ExecuteNode>::SharedPtr> clients_;
};

/// Register DetectObject / DetectPerson / RecognizeFace (perception conditions).
void register_perception(
  BT::BehaviorTreeFactory & f, std::shared_ptr<const PerceptionCache> cache,
  double max_age_sec);

/// True when `xml` uses a node of an extension that a dry run would skip (one not in `exempt`): decides whether to say
/// "dry run" for a plan. A plan made only of exempt (e.g. speech) nodes runs for real, so it is not a dry run.
bool plan_skips_nodes(
  const std::string & xml,
  const std::map<std::string, bt_pipeline_msgs::msg::ExtensionManifest> & manifests,
  const std::set<std::string> & exempt);

/// Register every Action/Condition node advertised in the extension manifests. Each runs
/// as a goal to `<extension>/execute`. Returns the number of registered nodes; Control and
/// Decorator manifest nodes are not supported and are reported through ctx->log.
int register_extensions(
  BT::BehaviorTreeFactory & f,
  const std::map<std::string, bt_pipeline_msgs::msg::ExtensionManifest> & manifests,
  std::shared_ptr<ExecContext> ctx);
}  // namespace bt_executor
