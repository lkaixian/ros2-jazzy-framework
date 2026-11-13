#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "bt_pipeline_msgs/msg/extension_manifest.hpp"
#include "bt_pipeline_msgs/srv/get_catalog.hpp"
#include "bt_pipeline_msgs/srv/llm_query.hpp"
#include "llm_bridge/failover.hpp"
#include "llm_bridge/llama_client.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using bt_pipeline_msgs::msg::ExtensionManifest;
using bt_pipeline_msgs::srv::GetCatalog;
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
    // Ordered failover list (first = preferred, e.g. Gemma E4B on the strong PC, then the small
    // model on the robot's laptop). Empty -> just server_url.
    declare_parameter("server_urls", std::vector<std::string>{});
    declare_parameter("connect_timeout_sec", 2.0);
    declare_parameter("failure_threshold", 2);
    declare_parameter("cooldown_sec", 15.0);
    declare_parameter("json_retries", 1);
    declare_parameter("probe_period_sec", 5.0);     // /health poll of every endpoint; 0 = off
    declare_parameter("startup_grace_sec", 60.0);   // no emergency while servers are loading
    declare_parameter("halt_on_all_down", true);    // emergency: halt the executor
    declare_parameter("halt_service", "bt_executor/halt");
    declare_parameter("model", "default");
    declare_parameter("temperature", 0.1);
    declare_parameter("max_tokens", 512);
    declare_parameter("timeout_sec", 60.0);
    declare_parameter("task_system_prompt", "");
    std::vector<std::string> urls;
    const std::vector<std::string> configured = get_parameter("server_urls").as_string_array();
    for (const auto & u : configured) {
      if (!u.empty()) {urls.push_back(u);}  // [""] is the "unset" value for YAML files
    }
    if (urls.empty()) {urls.push_back(get_parameter("server_url").as_string());}
    for (const auto & u : urls) {
      clients_.push_back(
        std::make_unique<llm_bridge::LlamaClient>(
          u, get_parameter("timeout_sec").as_double(),
          get_parameter("connect_timeout_sec").as_double()));
    }
    urls_ = urls;
    llm_bridge::FailoverConfig fc;
    fc.endpoints = urls.size();
    fc.max_failures = static_cast<int>(get_parameter("failure_threshold").as_int());
    fc.cooldown_sec = get_parameter("cooldown_sec").as_double();
    fc.json_retries = static_cast<int>(get_parameter("json_retries").as_int());
    failover_ = std::make_unique<llm_bridge::FailoverClient>(
      fc, [this](size_t i, const llm_bridge::ChatRequest & r) {return clients_[i]->chat(r);},
      [] {
        return std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      });
    started_ = now();

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
    rclcpp::QoS latched_status(1);
    latched_status.reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>("llm/status", latched_status);
    halt_client_ = create_client<std_srvs::srv::Trigger>(get_parameter("halt_service").as_string());
    const double probe = get_parameter("probe_period_sec").as_double();
    if (probe > 0.0) {
      probe_timer_ = create_wall_timer(
        std::chrono::duration<double>(probe), [this] {probe_once();}, work_group_);
    }
    // Default callback group on purpose: its response must be processed while on_task() waits.
    catalog_client_ = create_client<GetCatalog>("bt_validator/catalog");
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
        auto r = failover_->chat(cr);
        res->ok = r.chat.ok;
        res->text = r.chat.text;
        res->error = r.chat.error;
        report(r);
      }, rclcpp::ServicesQoS(), work_group_);
    for (size_t i = 0; i < urls_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "llm_bridge endpoint %zu%s -> %s", i, i ? " (fallback)" : "",
        urls_[i].c_str());
    }
    publish_state("ok", "starting");
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
    // The validator knows every usable node (core + extensions) and the detector's labels.
    std::string listing, labels;
    if (catalog_client_->wait_for_service(std::chrono::milliseconds(0))) {
      auto fut = catalog_client_->async_send_request(std::make_shared<GetCatalog::Request>());
      if (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        auto r = fut.get();
        listing = r->listing;
        for (size_t i = 0; i < r->labels.size(); ++i) {
          labels += (i ? ", " : "") + r->labels[i];
        }
      }
    }
    if (listing.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "bt_validator/catalog unavailable: prompt only lists extension nodes");
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (system.empty()) {system = kDefaultSystem;}
      if (!listing.empty()) {
        system += "Nodes (id[category](ports, ? = optional) - description):\n" + listing + "\n";
      } else {
        system += "Nodes:\n";
        for (const auto & kv : ext_listing_) {system += kv.second;}
      }
      if (!labels.empty()) {
        system += "Object labels the camera can detect: " + labels +
          ". For DetectObject use EXACTLY one of these labels; never invent one.\n";
      }
      scene = scene_;
    }
    auto cr = base_request();
    cr.system = system;
    cr.json_mode = true;
    cr.prompt = "Task: " + task + "\nScene: " + (scene.empty() ? "{}" : scene);
    auto r = failover_->chat(cr);
    report(r);
    if (!r.chat.ok) {
      RCLCPP_ERROR(get_logger(), "no usable LLM reply for task '%s': %s", task.c_str(),
        r.chat.error.c_str());
      if (r.all_down) {emergency("all LLM endpoints are down while a task was requested");}
      return;
    }
    std_msgs::msg::String out;
    out.data = r.chat.text;
    spec_pub_->publish(out);
  }

  double now() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
           .count();
  }

  void report(const llm_bridge::FailoverResult & r)
  {
    for (const auto & e : r.errors) {
      RCLCPP_WARN(get_logger(), "LLM attempt failed: %s", e.c_str());
    }
    if (r.chat.ok) {
      recovered();
      publish_state(r.endpoint == 0 ? "ok" : "degraded", "served by endpoint " +
        std::to_string(r.endpoint));
    }
  }

  // ok = every endpoint answers; degraded = only some do; down = none -> emergency.
  void publish_state(const std::string & state, const std::string & why)
  {
    if (state == last_state_) {return;}
    last_state_ = state;
    nlohmann::json j = {{"state", state}, {"reason", why}, {"endpoints", urls_}};
    std_msgs::msg::String m;
    m.data = j.dump();
    status_pub_->publish(m);
    if (state == "ok") {
      RCLCPP_INFO(get_logger(), "LLM state: ok (%s)", why.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "LLM state: %s (%s)", state.c_str(), why.c_str());
    }
  }

  void recovered()
  {
    if (emergency_active_) {
      RCLCPP_WARN(get_logger(), "an LLM endpoint is back: emergency cleared (robot stays halted "
        "until a new plan is submitted)");
    }
    emergency_active_ = false;
  }

  // Emergency procedure: report down, halt the BT executor (its extensions stop publishing, so
  // cmd_vel_mux zeroes the output within its watchdog). Repeated while the outage lasts, because
  // the first halt may find bt_executor not ready.
  void emergency(const std::string & why)
  {
    publish_state("down", why);
    if (!emergency_active_) {
      RCLCPP_ERROR(get_logger(), "EMERGENCY: %s", why.c_str());
    }
    emergency_active_ = true;
    if (!get_parameter("halt_on_all_down").as_bool()) {return;}
    if (!halt_client_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "EMERGENCY halt NOT delivered: %s is not available", halt_client_->get_service_name());
      return;
    }
    halt_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
        RCLCPP_ERROR(get_logger(), "EMERGENCY halt sent: %s", f.get()->message.c_str());
      });
  }

  void probe_once()
  {
    for (size_t i = 0; i < clients_.size(); ++i) {
      failover_->report_health(i, clients_[i]->healthy(2.0));
    }
    const auto av = failover_->available();
    size_t up = 0;
    for (bool a : av) {up += a ? 1 : 0;}
    if (up == av.size()) {
      recovered();
      publish_state("ok", "all endpoints healthy");
    } else if (up > 0) {
      recovered();
      publish_state("degraded", std::to_string(up) + " of " + std::to_string(av.size()) +
        " endpoints healthy");
    } else if (now() - started_ >= get_parameter("startup_grace_sec").as_double()) {
      emergency("every LLM endpoint failed its health check");
    }
  }

  std::vector<std::unique_ptr<llm_bridge::LlamaClient>> clients_;
  std::unique_ptr<llm_bridge::FailoverClient> failover_;
  std::vector<std::string> urls_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr halt_client_;
  rclcpp::TimerBase::SharedPtr probe_timer_;
  std::string last_state_;
  bool emergency_active_ = false;
  double started_ = 0.0;
  rclcpp::CallbackGroup::SharedPtr work_group_;
  rclcpp::Subscription<ExtensionManifest>::SharedPtr manifest_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_sub_, task_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spec_pub_;
  rclcpp::Service<LlmQuery>::SharedPtr query_srv_;
  rclcpp::Client<GetCatalog>::SharedPtr catalog_client_;
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
