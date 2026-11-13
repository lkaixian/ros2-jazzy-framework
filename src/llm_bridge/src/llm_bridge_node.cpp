#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "bt_pipeline_msgs/msg/extension_manifest.hpp"
#include "bt_pipeline_msgs/srv/llm_query.hpp"
#include "llm_bridge/llama_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using bt_pipeline_msgs::msg::ExtensionManifest;
using bt_pipeline_msgs::srv::LlmQuery;

namespace
{
const char * kDefaultSystem =
  "You convert a robot task into a JSON task spec. Output ONLY a JSON object: "
  "{\"goal\": string, \"retry\": optional int, \"steps\": [STEP,...]} where STEP is "
  "{\"node\": <id>, \"ports\": {<port>: <string>}} or a nested group "
  "{\"group\": \"Sequence|Fallback|ReactiveSequence|ReactiveFallback\", \"retry\": optional int, "
  "\"steps\": [STEP,...]}. Use Fallback for 'try A, otherwise B'. Use ONLY the node ids below "
  "with their ports (a trailing ? marks an optional port). Do not invent nodes.\n";
}

class LlmBridgeNode : public rclcpp::Node
{
public:
  LlmBridgeNode()
  : Node("llm_bridge_node")
  {
    declare_parameter("server_url", "http://127.0.0.1:8080");
    declare_parameter("model", "default");
    declare_parameter("temperature", 0.1);
    declare_parameter("max_tokens", 512);
    declare_parameter("timeout_sec", 60.0);
    declare_parameter("task_system_prompt", "");
    client_ = std::make_unique<llm_bridge::LlamaClient>(
      get_parameter("server_url").as_string(), get_parameter("timeout_sec").as_double());

    // Blocking HTTP calls live in their own group so manifests/scene keep flowing.
    work_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions work_opts;
    work_opts.callback_group = work_group_;

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    manifest_sub_ = create_subscription<ExtensionManifest>(
      "extensions/manifest", latched, [this](ExtensionManifest::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        std::ostringstream os;
        for (const auto & n : m->nodes) {
          os << n.id << "(";
          for (size_t i = 0; i < n.required_ports.size(); ++i) {
            os << (i ? "," : "") << n.required_ports[i];
          }
          for (const auto & p : n.optional_ports) {os << "," << p << "?";}
          os << ")\n";
        }
        ext_listing_[m->extension] = os.str();
      });
    scene_sub_ = create_subscription<std_msgs::msg::String>(
      "scene/summary", 1, [this](std_msgs::msg::String::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        scene_ = m->data;
      });
    spec_pub_ = create_publisher<std_msgs::msg::String>("bt/task_spec", 10);
    task_sub_ = create_subscription<std_msgs::msg::String>(
      "bt/task", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {on_task(m->data);},
      work_opts);
    query_srv_ = create_service<LlmQuery>(
      "llm/query",
      [this](const std::shared_ptr<LlmQuery::Request> req,
      std::shared_ptr<LlmQuery::Response> res) {
        llm_bridge::ChatRequest cr = base_request();
        cr.system = req->system;
        cr.prompt = req->prompt;
        cr.json_mode = req->json_mode;
        if (req->temperature > 0.0f) {cr.temperature = req->temperature;}
        if (req->max_tokens > 0) {cr.max_tokens = static_cast<int>(req->max_tokens);}
        auto r = client_->chat(cr);
        res->ok = r.ok;
        res->text = r.text;
        res->error = r.error;
      }, rclcpp::ServicesQoS(), work_group_);
    RCLCPP_INFO(get_logger(), "llm_bridge -> %s", get_parameter("server_url").as_string().c_str());
  }

private:
  llm_bridge::ChatRequest base_request()
  {
    llm_bridge::ChatRequest cr;
    cr.model = get_parameter("model").as_string();
    cr.temperature = get_parameter("temperature").as_double();
    cr.max_tokens = static_cast<int>(get_parameter("max_tokens").as_int());
    return cr;
  }

  void on_task(const std::string & task)
  {
    std::string system = get_parameter("task_system_prompt").as_string();
    std::string scene;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (system.empty()) {system = kDefaultSystem;}
      system += "Nodes:\n";
      for (const auto & kv : ext_listing_) {system += kv.second;}
      scene = scene_;
    }
    auto cr = base_request();
    cr.system = system;
    cr.json_mode = true;
    cr.prompt = "Task: " + task + "\nScene: " + (scene.empty() ? "{}" : scene);
    auto r = client_->chat(cr);
    if (!r.ok) {
      RCLCPP_ERROR(get_logger(), "llama-server: %s", r.error.c_str());
      return;
    }
    std_msgs::msg::String out;
    out.data = r.text;
    spec_pub_->publish(out);
  }

  std::unique_ptr<llm_bridge::LlamaClient> client_;
  rclcpp::CallbackGroup::SharedPtr work_group_;
  rclcpp::Subscription<ExtensionManifest>::SharedPtr manifest_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_sub_, task_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spec_pub_;
  rclcpp::Service<LlmQuery>::SharedPtr query_srv_;
  std::mutex mu_;
  std::map<std::string, std::string> ext_listing_;
  std::string scene_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LlmBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 2);
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
