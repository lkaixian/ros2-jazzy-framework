#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "bt_executor/registry.hpp"
#include "bt_pipeline_msgs/msg/bt_plan.hpp"
#include "nlohmann/json.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using bt_pipeline_msgs::msg::BtPlan;
using bt_pipeline_msgs::msg::ExtensionManifest;
using namespace std::chrono_literals;

// Remembers the last leaf node that returned FAILURE: Sequence/Fallback reset their children's
// status before tickOnce() returns, so the tree can no longer be asked afterwards.
class FailureTracker : public BT::StatusChangeLogger
{
public:
  explicit FailureTracker(BT::TreeNode * root)
  : BT::StatusChangeLogger(root) {}
  void callback(BT::Duration, const BT::TreeNode & n, BT::NodeStatus, BT::NodeStatus st) override
  {
    const auto t = n.type();
    if (st == BT::NodeStatus::FAILURE && t != BT::NodeType::CONTROL &&
      t != BT::NodeType::DECORATOR && t != BT::NodeType::SUBTREE)
    {
      last = n.registrationName();
      if (n.name() != last) {last += " (" + n.name() + ")";}
    }
  }
  void flush() override {}
  std::string last;
};

class BtExecutorNode : public rclcpp::Node
{
public:
  BtExecutorNode()
  : Node("bt_executor_node")
  {
    declare_parameter("dry_run", true);
    declare_parameter("auto_run", false);
    declare_parameter("tick_hz", 20.0);
    declare_parameter("max_runtime_sec", 300.0);
    declare_parameter("perception_max_age_sec", 1.0);
    cache_ = std::make_shared<bt_executor::PerceptionCache>();
    status_pub_ = create_publisher<std_msgs::msg::String>("bt/status", 10);

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    manifest_sub_ = create_subscription<ExtensionManifest>(
      "extensions/manifest", latched, [this](ExtensionManifest::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        manifests_[m->extension] = *m;
      });
    obj_sub_ = create_subscription<object_detection_msgr::msg::ObjectArray>(
      "objects", rclcpp::SensorDataQoS(),
      [this](object_detection_msgr::msg::ObjectArray::ConstSharedPtr m) {
        cache_->on_objects(*m, bt_executor::Clock::now());
      });
    per_sub_ = create_subscription<person_detection_msgr::msg::PersonArray>(
      "persons", rclcpp::SensorDataQoS(),
      [this](person_detection_msgr::msg::PersonArray::ConstSharedPtr m) {
        cache_->on_persons(*m, bt_executor::Clock::now());
      });
    face_sub_ = create_subscription<face_recognition_msgr::msg::FaceArray>(
      "faces", rclcpp::SensorDataQoS(),
      [this](face_recognition_msgr::msg::FaceArray::ConstSharedPtr m) {
        cache_->on_faces(*m, bt_executor::Clock::now());
      });
    plan_sub_ = create_subscription<BtPlan>(
      "bt/plan", 10, [this](BtPlan::ConstSharedPtr p) {on_plan(*p);});
    run_srv_ = create_service<std_srvs::srv::Trigger>(
      "bt_executor/run",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = start(res->message);
      });
    halt_srv_ = create_service<std_srvs::srv::Trigger>(
      "bt_executor/halt",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        halt_ = true;
        res->success = true;
        res->message = running_ ? "halt requested" : "nothing running";
      });
    RCLCPP_WARN_EXPRESSION(
      get_logger(), !get_parameter("dry_run").as_bool(),
      "dry_run=false: extension nodes WILL command hardware when a plan is run");
    status("idle", "ready; dry_run=" + std::string(get_parameter("dry_run").as_bool() ? "true" : "false"));
  }

  ~BtExecutorNode() override
  {
    halt_ = true;
    if (worker_.joinable()) {worker_.join();}
  }

private:
  // failed_node (optional): the leaf that failed, so a repair step knows where the plan broke
  void status(const std::string & state, const std::string & msg, const std::string & failed_node = "")
  {
    std_msgs::msg::String s;
    nlohmann::ordered_json j = {{"state", state}, {"message", msg}};   // keep state first
    if (!failed_node.empty()) {j["failed_node"] = failed_node;}
    s.data = j.dump();
    status_pub_->publish(s);
    RCLCPP_INFO(get_logger(), "%s: %s", state.c_str(), msg.c_str());
  }

  void on_plan(const BtPlan & p)
  {
    if (!p.valid) {
      RCLCPP_WARN(get_logger(), "ignoring plan with valid=false (%zu errors)", p.errors.size());
      return;
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      plan_xml_ = p.xml;
    }
    status("plan_received", "valid plan stored");
    if (get_parameter("auto_run").as_bool()) {
      std::string msg;
      if (!start(msg)) {RCLCPP_WARN(get_logger(), "auto_run: %s", msg.c_str());}
    }
  }

  bool start(std::string & msg)
  {
    std::string xml;
    std::map<std::string, ExtensionManifest> manifests;
    {
      std::lock_guard<std::mutex> lk(mu_);
      xml = plan_xml_;
      manifests = manifests_;
    }
    if (xml.empty()) {msg = "no valid plan received yet"; return false;}
    if (running_.exchange(true)) {msg = "a plan is already running"; return false;}
    if (worker_.joinable()) {worker_.join();}
    halt_ = false;
    worker_ = std::thread([this, xml, manifests]() {run(xml, manifests);});
    msg = "started";
    return true;
  }

  void run(const std::string & xml, const std::map<std::string, ExtensionManifest> & manifests)
  {
    const bool dry = get_parameter("dry_run").as_bool();
    auto ctx = std::make_shared<bt_executor::ExecContext>();
    ctx->node = shared_from_this();
    ctx->dry_run = dry;
    ctx->log = [this](const std::string & s) {RCLCPP_INFO(get_logger(), "%s", s.c_str());};
    std::string result = "FAILURE";
    std::string failed;
    try {
      BT::BehaviorTreeFactory factory;
      bt_executor::register_perception(
        factory, cache_, get_parameter("perception_max_age_sec").as_double());
      bt_executor::register_extensions(factory, manifests, ctx);
      auto tree = factory.createTreeFromText(xml);
      FailureTracker tracker(tree.rootNode());
      status("running", dry ? "dry run" : "LIVE");
      const auto period = std::chrono::duration<double>(1.0 / get_parameter("tick_hz").as_double());
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(get_parameter("max_runtime_sec").as_double());
      BT::NodeStatus st = BT::NodeStatus::RUNNING;
      while (st == BT::NodeStatus::RUNNING || st == BT::NodeStatus::IDLE) {
        if (halt_) {result = "HALTED"; break;}
        if (std::chrono::steady_clock::now() > deadline) {result = "TIMEOUT"; break;}
        st = tree.tickOnce();
        if (st == BT::NodeStatus::RUNNING) {
          tree.sleep(std::chrono::duration_cast<std::chrono::milliseconds>(period));
        }
      }
      if (result == "HALTED" || result == "TIMEOUT") {
        tree.haltTree();     // cancels in-flight extension goals
      } else {
        result = BT::toStr(st);
      }
      if (result == "FAILURE") {failed = tracker.last;}
    } catch (const std::exception & e) {
      result = std::string("ERROR: ") + e.what();
    }
    status("finished", result, failed);
    running_ = false;
  }

  std::shared_ptr<bt_executor::PerceptionCache> cache_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<ExtensionManifest>::SharedPtr manifest_sub_;
  rclcpp::Subscription<object_detection_msgr::msg::ObjectArray>::SharedPtr obj_sub_;
  rclcpp::Subscription<person_detection_msgr::msg::PersonArray>::SharedPtr per_sub_;
  rclcpp::Subscription<face_recognition_msgr::msg::FaceArray>::SharedPtr face_sub_;
  rclcpp::Subscription<BtPlan>::SharedPtr plan_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_srv_, halt_srv_;
  std::mutex mu_;
  std::string plan_xml_;
  std::map<std::string, ExtensionManifest> manifests_;
  std::thread worker_;
  std::atomic<bool> running_{false}, halt_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtExecutorNode>();
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 3);
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
