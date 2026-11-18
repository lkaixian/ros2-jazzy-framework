#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
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
  void callback(BT::Duration, const BT::TreeNode & n, BT::NodeStatus prev, BT::NodeStatus st) override
  {
    // sticky for the dashboard: Sequence/Fallback reset finished children to IDLE, which would erase what happened
    if (st != BT::NodeStatus::IDLE) {
      state[n.UID()] = st;
      dirty = true;
    } else if (prev == BT::NodeStatus::RUNNING) {   // halted while running (e.g. a Parallel finished first)
      state[n.UID()] = BT::NodeStatus::SKIPPED;
      dirty = true;
    }
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
  std::map<uint16_t, BT::NodeStatus> state;
  bool dirty = false;
};

// The tree as JSON for the dashboard: [{uid, depth, parent, type, name, ports{}}] in depth-first order.
static nlohmann::ordered_json describe_tree(const BT::TreeNode * root)
{
  nlohmann::ordered_json out = nlohmann::ordered_json::array();
  std::function<void(const BT::TreeNode *, int, int)> walk = [&](const BT::TreeNode * n, int depth, int parent) {
      nlohmann::ordered_json j = {{"uid", n->UID()}, {"depth", depth}, {"parent", parent},
        {"type", BT::toStr(n->type())}, {"name", n->registrationName()}};
      nlohmann::ordered_json ports = nlohmann::ordered_json::object();
      for (const auto & [k, v] : n->config().input_ports) {ports[k] = v;}
      for (const auto & [k, v] : n->config().output_ports) {ports[k] = v;}
      if (!ports.empty()) {j["ports"] = ports;}
      out.push_back(j);
      if (auto * c = dynamic_cast<const BT::ControlNode *>(n)) {
        for (const auto * ch : c->children()) {walk(ch, depth + 1, n->UID());}
      } else if (auto * d = dynamic_cast<const BT::DecoratorNode *>(n)) {
        if (d->child()) {walk(d->child(), depth + 1, n->UID());}
      }
    };
  walk(root, 0, -1);
  return out;
}

class BtExecutorNode : public rclcpp::Node
{
public:
  BtExecutorNode()
  : Node("bt_executor_node")
  {
    declare_parameter("dry_run", true);
    declare_parameter("dry_run_exempt", std::vector<std::string>{});   // extensions that run for real in a dry run
    declare_parameter("auto_run", false);
    declare_parameter("tick_hz", 20.0);
    declare_parameter("max_runtime_sec", 300.0);
    declare_parameter("perception_max_age_sec", 1.0);
    declare_parameter("mode_gate", false);       // refuse to start a run unless a fresh robot/mode says run_ok (fails closed)
    declare_parameter("mode_gate_max_age_sec", 3.0);
    declare_parameter("groot2", false);          // serve the running tree to Groot2 (Monitor tab, port groot2_port)
    declare_parameter("groot2_port", 1667);
    declare_parameter("nodes_model_path", "");   // when set, write the node palette (all registered nodes) there for Groot2
    cache_ = std::make_shared<bt_executor::PerceptionCache>();
    status_pub_ = create_publisher<std_msgs::msg::String>("bt/status", 10);
    rclcpp::QoS latched_tree(1);
    latched_tree.reliable().transient_local();
    tree_pub_ = create_publisher<std_msgs::msg::String>("bt/tree", latched_tree);   // structure of the running tree
    node_status_pub_ = create_publisher<std_msgs::msg::String>("bt/node_status", 10);   // {uid: R|S|F|I}, on change

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
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "robot/mode", latched, [this](std_msgs::msg::String::ConstSharedPtr m) {
        const auto j = nlohmann::json::parse(m->data, nullptr, false);
        std::lock_guard<std::mutex> lk(mu_);
        mode_ok_ = j.is_object() && j.value("run_ok", false);
        mode_why_ = j.is_object() ? j.value("mode", std::string("?")) : std::string("unreadable");
        mode_t_ = std::chrono::steady_clock::now();
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
    if (get_parameter("mode_gate").as_bool()) {
      std::lock_guard<std::mutex> lk(mu_);
      const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - mode_t_).count();
      if (!mode_seen() || age > get_parameter("mode_gate_max_age_sec").as_double()) {
        msg = "mode gate: no fresh robot/mode (is robot_mode_node running?)";
        return false;
      }
      if (!mode_ok_) {msg = "mode gate: robot mode is " + mode_why_; return false;}
    }
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
    const std::vector<std::string> exempt = get_parameter("dry_run_exempt").as_string_array();   // a copy: the Parameter is a temporary
    ctx->dry_run_exempt.insert(exempt.begin(), exempt.end());
    ctx->log = [this](const std::string & s) {RCLCPP_INFO(get_logger(), "%s", s.c_str());};
    std::string result = "FAILURE";
    std::string failed;
    try {
      BT::BehaviorTreeFactory factory;
      bt_executor::register_perception(
        factory, cache_, get_parameter("perception_max_age_sec").as_double());
      bt_executor::register_extensions(factory, manifests, ctx);
      auto tree = factory.createTreeFromText(xml);
      const auto model_path = get_parameter("nodes_model_path").as_string();
      if (!model_path.empty()) {
        std::ofstream(model_path) << BT::writeTreeNodesModelXML(factory);
      }
      std::unique_ptr<BT::Groot2Publisher> groot;     // lives for this run only: Groot2 reconnects for each plan
      if (get_parameter("groot2").as_bool()) {
        try {
          groot = std::make_unique<BT::Groot2Publisher>(tree, static_cast<unsigned>(get_parameter("groot2_port").as_int()));
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Groot2 publisher not started: %s", e.what());   // e.g. port busy: the run goes on
        }
      }
      FailureTracker tracker(tree.rootNode());
      {
        std_msgs::msg::String t;
        t.data = describe_tree(tree.rootNode()).dump();
        tree_pub_->publish(t);
      }
      auto last_pub = std::chrono::steady_clock::now();
      auto publish_nodes = [&](bool force) {
          const auto now = std::chrono::steady_clock::now();
          if (!tracker.dirty || (!force && now - last_pub < std::chrono::milliseconds(100))) {return;}
          nlohmann::ordered_json j = nlohmann::ordered_json::object();
          for (const auto & [uid, st] : tracker.state) {
            j[std::to_string(uid)] = st == BT::NodeStatus::RUNNING ? "R" : st == BT::NodeStatus::SUCCESS ? "S" :
              st == BT::NodeStatus::FAILURE ? "F" : st == BT::NodeStatus::SKIPPED ? "K" : "I";
          }
          std_msgs::msg::String m;
          m.data = nlohmann::ordered_json{{"s", j}}.dump();
          node_status_pub_->publish(m);
          tracker.dirty = false;
          last_pub = now;
        };
      // "dry run" only when this plan has nodes that are skipped; a plan of exempt (speech) nodes simply runs
      status("running", !dry ? "LIVE" : bt_executor::plan_skips_nodes(xml, manifests, ctx->dry_run_exempt) ? "dry run" : "running");
      const auto period = std::chrono::duration<double>(1.0 / get_parameter("tick_hz").as_double());
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(get_parameter("max_runtime_sec").as_double());
      BT::NodeStatus st = BT::NodeStatus::RUNNING;
      while (st == BT::NodeStatus::RUNNING || st == BT::NodeStatus::IDLE) {
        if (halt_) {result = "HALTED"; break;}
        if (std::chrono::steady_clock::now() > deadline) {result = "TIMEOUT"; break;}
        st = tree.tickOnce();
        publish_nodes(false);
        if (st == BT::NodeStatus::RUNNING) {
          tree.sleep(std::chrono::duration_cast<std::chrono::milliseconds>(period));
        }
      }
      if (result == "HALTED" || result == "TIMEOUT") {
        tree.haltTree();     // cancels in-flight extension goals
      } else {
        result = BT::toStr(st);
      }
      publish_nodes(true);
      if (result == "FAILURE") {failed = tracker.last;}
    } catch (const std::exception & e) {
      result = std::string("ERROR: ") + e.what();
    }
    status("finished", result, failed);
    running_ = false;
  }

  std::shared_ptr<bt_executor::PerceptionCache> cache_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, tree_pub_, node_status_pub_;
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  bool mode_ok_{false};
  std::string mode_why_;
  std::chrono::steady_clock::time_point mode_t_{};     // default (epoch) = never heard
  bool mode_seen() const {return mode_t_ != std::chrono::steady_clock::time_point{};}
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
