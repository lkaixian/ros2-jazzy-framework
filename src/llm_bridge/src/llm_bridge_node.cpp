#include <atomic>
#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "bt_pipeline_msgs/msg/extension_manifest.hpp"
#include "bt_pipeline_msgs/srv/get_catalog.hpp"
#include "bt_pipeline_msgs/srv/llm_query.hpp"
#include "cluster_msgs/msg/node_info.hpp"
#include "llm_bridge/failover.hpp"
#include "llm_bridge/llama_client.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/version.h"

// Humble (rclcpp 16) takes rmw_qos_profile_t for services; newer distros take rclcpp::QoS.
#if RCLCPP_VERSION_MAJOR >= 21
#define SERVICE_QOS rclcpp::ServicesQoS()
#else
#define SERVICE_QOS rmw_qos_profile_services_default
#endif

using bt_pipeline_msgs::msg::ExtensionManifest;
using bt_pipeline_msgs::srv::GetCatalog;
using bt_pipeline_msgs::srv::LlmQuery;

namespace
{
const char * kDefaultSystem =
  "You convert a robot task into a JSON task spec. Output ONLY a JSON object: "
  "{\"goal\": string, \"retry\": optional int, \"steps\": [STEP,...]} where STEP is a macro call "
  "{\"macro\": <name>, \"args\": {<arg>: <string>}}, or {\"node\": <id>, \"ports\": {<port>: <string>}}, "
  "or a nested group {\"group\": \"Sequence|Fallback|ReactiveSequence|ReactiveFallback|Parallel\", "
  "\"retry\": optional int, \"repeat\": optional int (run the group N times in a row), \"steps\": [STEP,...]} (Parallel runs its steps at the same time, e.g. speak while driving; \"success_count\": N = how many must succeed), or a loop {\"foreach\": [value,...], \"as\": \"x\", \"steps\": [STEP using ${x}]} that repeats the steps once per value (max 8). PREFER a macro whenever one fits the request "
  "(it is shorter and tested); use single nodes only for what no macro does. Keep the plan short "
  "(usually 1 to 5 steps) and do exactly what was asked. If the robot cannot do the task (see the "
  "robot knowledge), do not invent steps: use the say macro to tell the person so. Use Fallback "
  "for 'try A, otherwise B'. Use ONLY the macros and node ids below with their args/ports (a "
  "trailing ? marks an optional one). Nodes are tagged sync (finishes inside one tick) or async "
  "(runs over many ticks and may fail halfway: movement, speech, anything over DDS). If a REPAIR "
  "note says where the previous plan broke, change that step and keep the rest.\n";
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
    // Hosts advertise their running LLM servers in cluster/node_info; a host that joins is added
    // to the endpoint list and one that goes silent for endpoint_ttl_sec is removed. Static
    // server_urls always stay (typically the local small model) after every advertised endpoint.
    declare_parameter("use_cluster", true);
    declare_parameter("endpoint_kind", "llm");
    declare_parameter("endpoint_ttl_sec", 4.0);   // < scheduler failover_after_sec (5 s)
    declare_parameter("static_priority_base", 100);
    declare_parameter("use_static_urls", true);   // false: only cluster-advertised endpoints
    // Robot facts the node catalog cannot say (limits, places, house rules), appended to the
    // planning prompt. "" = framework_bringup/param/llm_knowledge.md if installed.
    declare_parameter("knowledge_file", "");
    // Name of the ENVIRONMENT VARIABLE that holds the llama-server API key (never put the key
    // itself in a YAML file or on a command line). Unset/empty variable = unauthenticated.
    declare_parameter("api_key_env", "LLM_API_KEY");
    declare_parameter("model", "default");
    declare_parameter("temperature", 0.1);
    declare_parameter("max_tokens", 512);
    declare_parameter("repeat_penalty", 0.0);   // > 0 is sent to the server; off: the earlier loops were a schema key-order bug, not sampling
    declare_parameter("timeout_sec", 60.0);
    declare_parameter("task_system_prompt", "");
    std::vector<std::string> urls;
    const std::vector<std::string> configured = get_parameter("server_urls").as_string_array();
    for (const auto & u : configured) {
      if (!u.empty()) {urls.push_back(u);}  // [""] is the "unset" value for YAML files
    }
    if (urls.empty()) {urls.push_back(get_parameter("server_url").as_string());}
    if (get_parameter("use_static_urls").as_bool()) {static_urls_ = urls;}
    llm_bridge::FailoverConfig fc;
    fc.max_failures = static_cast<int>(get_parameter("failure_threshold").as_int());
    fc.cooldown_sec = get_parameter("cooldown_sec").as_double();
    fc.json_retries = static_cast<int>(get_parameter("json_retries").as_int());
    failover_ = std::make_unique<llm_bridge::FailoverClient>(
      fc, [this](const std::string & u, const llm_bridge::ChatRequest & r) {
        return client_for(u).chat(r);
      },
      [] {
        return std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      });
    started_ = now();
    {
      const char * k = std::getenv(get_parameter("api_key_env").as_string().c_str());
      api_key_ = k ? k : "";
      if (api_key_.empty()) {
        RCLCPP_WARN(get_logger(), "no API key in $%s: requests to the LLM servers are "
          "unauthenticated (set it, and start llama-server with the same key)",
          get_parameter("api_key_env").as_string().c_str());
      } else {
        RCLCPP_INFO(get_logger(), "LLM requests carry an API key from $%s",
          get_parameter("api_key_env").as_string().c_str());
      }
    }
    knowledge_ = load_knowledge();
    refresh_endpoints();

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
    err_pub_ = create_publisher<std_msgs::msg::String>("llm/error", 10);
    rclcpp::QoS latched_status(1);
    latched_status.reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>("llm/status", latched_status);
    halt_client_ = create_client<std_srvs::srv::Trigger>(get_parameter("halt_service").as_string());
    if (get_parameter("use_cluster").as_bool()) {
      cluster_sub_ = create_subscription<cluster_msgs::msg::NodeInfo>(
        "cluster/node_info", rclcpp::QoS(10).reliable(),
        [this](cluster_msgs::msg::NodeInfo::ConstSharedPtr m) {
          {
            std::lock_guard<std::mutex> lk(mu_);
            auto & h = hosts_[m->host_id];
            h.rx = now();
            h.eps.clear();
            for (const auto & e : m->endpoints) {
              if (e.kind == get_parameter("endpoint_kind").as_string()) {
                h.eps.push_back({e.url, e.priority});
              }
            }
          }
          refresh_endpoints();
        });
    }
    // leaves are noticed by silence, so re-evaluate even when no heartbeat arrives
    refresh_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {refresh_endpoints();});
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
    // the orchestrator asks for a repaired plan here; to the LLM it is just another task
    repair_sub_ = create_subscription<std_msgs::msg::String>(
      "bt/repair", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {on_task(m->data);},
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
      }, SERVICE_QOS, work_group_);
    for (const auto & u : static_urls_) {
      RCLCPP_INFO(get_logger(), "llm_bridge static endpoint -> %s", u.c_str());
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
    cr.repeat_penalty = get_parameter("repeat_penalty").as_double();
    return cr;
  }

  void on_task(const std::string & task)
  {
    std::string system = get_parameter("task_system_prompt").as_string();
    std::string scene;
    // The validator knows every usable node (core + extensions) and the detector's labels.
    std::string listing, labels, schema;
    if (catalog_client_->wait_for_service(std::chrono::milliseconds(0))) {
      auto fut = catalog_client_->async_send_request(std::make_shared<GetCatalog::Request>());
      if (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        auto r = fut.get();
        listing = r->listing;
        schema = r->schema;
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
      if (!knowledge_.empty()) {system += "Robot knowledge:\n" + knowledge_ + "\n";}
      scene = scene_;
    }
    auto cr = base_request();
    cr.system = system;
    cr.json_mode = true;
    cr.schema = schema;
    cr.prompt = "Task: " + task + "\nScene: " + (scene.empty() ? "{}" : scene);
    auto r = failover_->chat(cr);
    report(r);
    if (!r.chat.ok) {
      RCLCPP_ERROR(get_logger(), "no usable LLM reply for task '%s': %s", task.c_str(),
        r.chat.error.c_str());
      std_msgs::msg::String err;   // the orchestrator turns this into an invalid plan, so the task does not vanish
      err.data = nlohmann::json{{"task", task}, {"error", r.chat.error}}.dump();
      err_pub_->publish(err);
      if (r.all_down) {emergency("all LLM endpoints are down while a task was requested");}
      return;
    }
    std_msgs::msg::String out;
    out.data = r.chat.text;
    spec_pub_->publish(out);
  }

  std::string load_knowledge()
  {
    std::string path = get_parameter("knowledge_file").as_string();
    if (path.empty()) {
      try {
        path = ament_index_cpp::get_package_share_directory("framework_bringup") +
          "/param/llm_knowledge.md";
      } catch (const std::exception &) {
        RCLCPP_WARN(get_logger(), "framework_bringup not installed: planning without robot knowledge");
        return "";
      }
    }
    std::ifstream f(path);
    if (!f) {
      RCLCPP_WARN(get_logger(), "knowledge file %s not readable: planning without it", path.c_str());
      return "";
    }
    std::ostringstream os;
    os << f.rdbuf();
    RCLCPP_INFO(get_logger(), "robot knowledge loaded from %s (%zu bytes)", path.c_str(), os.str().size());
    return os.str();
  }

  double now() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
           .count();
  }

  llm_bridge::LlamaClient & client_for(const std::string & url)
  {
    std::lock_guard<std::mutex> lk(clients_mu_);
    auto & c = clients_[url];
    if (!c) {
      c = std::make_unique<llm_bridge::LlamaClient>(
        url, get_parameter("timeout_sec").as_double(),
        get_parameter("connect_timeout_sec").as_double(), api_key_);
    }
    return *c;
  }

  // advertised endpoints of live hosts (best priority first) + the static fallbacks
  void refresh_endpoints()
  {
    std::vector<llm_bridge::Endpoint> eps;
    {
      std::lock_guard<std::mutex> lk(mu_);
      const double ttl = get_parameter("endpoint_ttl_sec").as_double();
      const double t = now();
      for (auto it = hosts_.begin(); it != hosts_.end();) {
        if (t - it->second.rx > ttl) {
          RCLCPP_WARN(get_logger(), "host %s silent for %.1f s: its LLM endpoints leave",
            it->first.c_str(), t - it->second.rx);
          it = hosts_.erase(it);
          continue;
        }
        for (const auto & e : it->second.eps) {eps.push_back({e.url, e.priority});}
        ++it;
      }
    }
    const int base = static_cast<int>(get_parameter("static_priority_base").as_int());
    for (size_t i = 0; i < static_urls_.size(); ++i) {
      eps.push_back({static_urls_[i], base + static_cast<int>(i)});
    }
    // after the first call, newly advertised endpoints wait for a passing /health (needs the prober)
    const bool probation = initialized_ && get_parameter("probe_period_sec").as_double() > 0.0;
    initialized_ = true;
    auto delta = failover_->set_endpoints(eps, probation);
    for (const auto & u : delta.first) {RCLCPP_INFO(get_logger(), "LLM endpoint joined: %s", u.c_str());}
    for (const auto & u : delta.second) {RCLCPP_WARN(get_logger(), "LLM endpoint left: %s", u.c_str());}
    if (!delta.first.empty() || !delta.second.empty()) {
      std::string cur;
      {
        std::lock_guard<std::mutex> lk(state_mu_);
        cur = last_state_;
      }
      if (!cur.empty()) {publish_state(cur, "endpoint set changed");}
    }
  }

  void report(const llm_bridge::FailoverResult & r)
  {
    for (const auto & e : r.errors) {
      RCLCPP_WARN(get_logger(), "LLM attempt failed: %s", e.c_str());
    }
    if (r.chat.ok) {
      recovered();
      publish_state(r.rank == 0 ? "ok" : "degraded", "served by " + r.endpoint);
    }
  }

  // ok = every endpoint answers; degraded = only some do; down = none -> emergency.
  void publish_state(const std::string & state, const std::string & why)
  {
    nlohmann::json eps = nlohmann::json::array();
    for (const auto & kv : failover_->status()) {
      eps.push_back({{"url", kv.first}, {"available", kv.second}});
    }
    // republish on a state change OR when the endpoint set / availability changed
    const std::string sig = eps.dump();
    std::lock_guard<std::mutex> lk(state_mu_);   // called from several callback groups
    if (state == last_state_ && sig == last_sig_) {return;}
    last_state_ = state;
    last_sig_ = sig;
    nlohmann::json j = {{"state", state}, {"reason", why}, {"endpoints", eps}};
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
    refresh_endpoints();
    for (const auto & kv : failover_->status()) {
      failover_->report_health(kv.first, client_for(kv.first).healthy(2.0));
    }
    const auto av = failover_->status();
    size_t up = 0;
    for (const auto & kv : av) {up += kv.second ? 1 : 0;}
    if (!av.empty() && up == av.size()) {
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

  struct HostEps
  {
    double rx = 0.0;
    std::vector<llm_bridge::Endpoint> eps;
  };
  std::map<std::string, std::unique_ptr<llm_bridge::LlamaClient>> clients_;
  std::mutex clients_mu_;
  std::map<std::string, HostEps> hosts_;   // guarded by mu_
  rclcpp::Subscription<cluster_msgs::msg::NodeInfo>::SharedPtr cluster_sub_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  std::unique_ptr<llm_bridge::FailoverClient> failover_;
  std::vector<std::string> static_urls_;
  std::string knowledge_;
  std::string api_key_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr halt_client_;
  rclcpp::TimerBase::SharedPtr probe_timer_;
  std::atomic<bool> initialized_{false};
  std::mutex state_mu_;
  std::string last_state_, last_sig_;   // guarded by state_mu_
  std::atomic<bool> emergency_active_{false};
  double started_ = 0.0;
  rclcpp::CallbackGroup::SharedPtr work_group_;
  rclcpp::Subscription<ExtensionManifest>::SharedPtr manifest_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_sub_, task_sub_, repair_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spec_pub_, err_pub_;
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
