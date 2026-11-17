#include "bt_executor/registry.hpp"

#include <set>
#include <sstream>

namespace bt_executor
{
namespace
{
double age(Clock::time_point now, Clock::time_point then)
{
  return std::chrono::duration<double>(now - then).count();
}

double to_double(const BT::TreeNode & n, const std::string & port, double dflt)
{
  auto v = n.getInput<std::string>(port);
  if (!v || v.value().empty()) {return dflt;}
  try {
    return std::stod(v.value());
  } catch (const std::exception &) {
    return dflt;
  }
}

bool port_in_xml(const BT::TreeNode & n, const std::string & port)
{
  return n.config().output_ports.count(port) > 0;
}
}  // namespace

// ---------------- PerceptionCache ----------------
void PerceptionCache::on_objects(
  const object_detection_msgr::msg::ObjectArray & m, Clock::time_point now)
{
  std::lock_guard<std::mutex> lk(mu_);
  objects_ = m; t_obj_ = now; has_obj_ = true;
}
void PerceptionCache::on_persons(
  const person_detection_msgr::msg::PersonArray & m, Clock::time_point now)
{
  std::lock_guard<std::mutex> lk(mu_);
  persons_ = m; t_per_ = now; has_per_ = true;
}
void PerceptionCache::on_faces(
  const face_recognition_msgr::msg::FaceArray & m, Clock::time_point now)
{
  std::lock_guard<std::mutex> lk(mu_);
  faces_ = m; t_face_ = now; has_face_ = true;
}

bool PerceptionCache::object(
  const std::string & label, double min_conf, double max_age_sec, Clock::time_point now,
  std::string * bbox_out) const
{
  std::lock_guard<std::mutex> lk(mu_);
  if (!has_obj_ || age(now, t_obj_) > max_age_sec) {return false;}
  for (const auto & o : objects_.objects) {
    if (o.label == label && o.confidence >= min_conf) {
      if (bbox_out) {
        std::ostringstream os;
        os << o.x_min << "," << o.y_min << "," << o.x_max << "," << o.y_max;
        *bbox_out = os.str();
      }
      return true;
    }
  }
  return false;
}

bool PerceptionCache::person(
  double min_conf, double max_age_sec, Clock::time_point now, int * track_out) const
{
  std::lock_guard<std::mutex> lk(mu_);
  if (!has_per_ || age(now, t_per_) > max_age_sec) {return false;}
  for (const auto & p : persons_.persons) {
    if (p.confidence >= min_conf) {
      if (track_out) {*track_out = p.track_id;}
      return true;
    }
  }
  return false;
}

bool PerceptionCache::face(
  const std::string & identity, double min_similarity, double max_age_sec,
  Clock::time_point now) const
{
  std::lock_guard<std::mutex> lk(mu_);
  if (!has_face_ || age(now, t_face_) > max_age_sec) {return false;}
  for (const auto & f : faces_.faces) {
    const bool who = identity.empty() ? f.identity != "unknown" : f.identity == identity;
    if (who && f.similarity >= min_similarity) {return true;}
  }
  return false;
}

bool PerceptionCache::face_feature(
  double min_conf, double max_age_sec, Clock::time_point now, std::string * identity,
  double * similarity) const
{
  std::lock_guard<std::mutex> lk(mu_);
  if (!has_face_ || age(now, t_face_) > max_age_sec) {return false;}
  const face_recognition_msgr::msg::Face * best = nullptr;
  double best_area = -1.0;
  for (const auto & f : faces_.faces) {
    if (f.confidence < min_conf || f.embedding.empty()) {continue;}
    const double area = static_cast<double>(f.x_max - f.x_min) * (f.y_max - f.y_min);
    if (area > best_area) {best_area = area; best = &f;}
  }
  if (!best) {return false;}
  if (identity) {*identity = best->identity;}
  if (similarity) {*similarity = best->similarity;}
  return true;
}

// ---------------- ExecContext ----------------
rclcpp_action::Client<ExecuteNode>::SharedPtr ExecContext::client(const std::string & ext)
{
  std::lock_guard<std::mutex> lk(mu_);
  auto & c = clients_[ext];
  if (!c) {c = rclcpp_action::create_client<ExecuteNode>(node, ext + "/execute");}
  return c;
}

// ---------------- perception conditions ----------------
void register_perception(
  BT::BehaviorTreeFactory & f, std::shared_ptr<const PerceptionCache> cache, double max_age)
{
  f.registerSimpleCondition(
    "DetectObject", [cache, max_age](BT::TreeNode & n) {
      auto label = n.getInput<std::string>("label");
      if (!label || label.value().empty()) {return BT::NodeStatus::FAILURE;}
      std::string bbox;
      if (!cache->object(label.value(), to_double(n, "min_conf", 0.5), max_age, Clock::now(),
      &bbox)) {return BT::NodeStatus::FAILURE;}
      if (port_in_xml(n, "out_bbox")) {n.setOutput("out_bbox", bbox);}
      return BT::NodeStatus::SUCCESS;
    },
    {BT::InputPort<std::string>("label"), BT::InputPort<std::string>("min_conf"),
      BT::OutputPort<std::string>("out_bbox")});

  f.registerSimpleCondition(
    "DetectPerson", [cache, max_age](BT::TreeNode & n) {
      int tid = -1;
      if (!cache->person(to_double(n, "min_conf", 0.5), max_age, Clock::now(), &tid)) {
        return BT::NodeStatus::FAILURE;
      }
      if (port_in_xml(n, "out_track_id")) {n.setOutput("out_track_id", std::to_string(tid));}
      return BT::NodeStatus::SUCCESS;
    },
    {BT::InputPort<std::string>("min_conf"), BT::OutputPort<std::string>("out_track_id")});

  f.registerSimpleCondition(
    "RecognizeFace", [cache, max_age](BT::TreeNode & n) {
      auto id = n.getInput<std::string>("identity");
      return cache->face(id ? id.value() : "", to_double(n, "min_similarity", 0.6), max_age,
             Clock::now()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    },
    {BT::InputPort<std::string>("identity"), BT::InputPort<std::string>("min_similarity")});

  f.registerSimpleCondition(
    "ExtractFaceFeature", [cache, max_age](BT::TreeNode & n) {
      std::string who;
      double sim = 0.0;
      if (!cache->face_feature(to_double(n, "min_conf", 0.5), max_age, Clock::now(), &who, &sim)) {
        return BT::NodeStatus::FAILURE;
      }
      if (port_in_xml(n, "out_identity")) {n.setOutput("out_identity", who);}
      if (port_in_xml(n, "out_similarity")) {
        std::ostringstream os;
        os.precision(3);
        os << sim;
        n.setOutput("out_similarity", os.str());
      }
      return BT::NodeStatus::SUCCESS;
    },
    {BT::InputPort<std::string>("min_conf"), BT::OutputPort<std::string>("out_identity"),
      BT::OutputPort<std::string>("out_similarity")});
}

// ---------------- extension nodes ----------------
namespace
{
struct GoalState
{
  std::mutex mu;
  rclcpp_action::ClientGoalHandle<ExecuteNode>::SharedPtr handle;
  bool rejected = false, done = false, success = false;
  std::string message;
  std::vector<std::string> out_names, out_values;
};

class ExtNode : public BT::StatefulActionNode
{
public:
  ExtNode(
    const std::string & name, const BT::NodeConfig & cfg, std::shared_ptr<ExecContext> ctx,
    std::string ext, std::string id, std::vector<std::string> ports, std::set<std::string> outs)
  : BT::StatefulActionNode(name, cfg), ctx_(std::move(ctx)), ext_(std::move(ext)),
    id_(std::move(id)), ports_(std::move(ports)), outs_(std::move(outs)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    ExecuteNode::Goal goal;
    goal.node_id = id_;
    for (const auto & p : ports_) {
      auto v = getInput<std::string>(p);
      if (v && !v.value().empty()) {
        goal.port_names.push_back(p);
        goal.port_values.push_back(v.value());
      }
    }
    std::ostringstream desc;
    desc << ext_ << "/" << id_ << "(";
    for (size_t i = 0; i < goal.port_names.size(); ++i) {
      desc << (i ? ", " : "") << goal.port_names[i] << "=" << goal.port_values[i];
    }
    desc << ")";
    if (ctx_->dry_run && !ctx_->dry_run_exempt.count(ext_)) {
      ctx_->log("[dry-run] " + desc.str());
      return BT::NodeStatus::SUCCESS;
    }
    if (!ctx_->node) {return BT::NodeStatus::FAILURE;}
    client_ = ctx_->client(ext_);
    if (!client_->action_server_is_ready()) {
      ctx_->log("extension not available: " + ext_ + " (" + desc.str() + ")");
      return BT::NodeStatus::FAILURE;
    }
    state_ = std::make_shared<GoalState>();
    auto st = state_;
    rclcpp_action::Client<ExecuteNode>::SendGoalOptions opts;
    opts.goal_response_callback = [st](auto gh) {
        std::lock_guard<std::mutex> lk(st->mu);
        if (!gh) {st->rejected = true;} else {st->handle = gh;}
      };
    opts.result_callback = [st](const rclcpp_action::ClientGoalHandle<ExecuteNode>::WrappedResult & r) {
        std::lock_guard<std::mutex> lk(st->mu);
        st->done = true;
        st->success = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
          r.result->success;
        if (r.result) {
          st->message = r.result->message;
          st->out_names = r.result->output_names;
          st->out_values = r.result->output_values;
        }
      };
    client_->async_send_goal(goal, opts);
    ctx_->log("start " + desc.str());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lk(state_->mu);
    if (state_->rejected) {
      ctx_->log(id_ + " rejected by " + ext_);
      return BT::NodeStatus::FAILURE;
    }
    if (!state_->done) {return BT::NodeStatus::RUNNING;}
    ctx_->log(id_ + (state_->success ? " succeeded: " : " failed: ") + state_->message);
    if (state_->success) {   // out_<name> ports named in the XML land on the blackboard
      for (size_t i = 0; i < state_->out_names.size() && i < state_->out_values.size(); ++i) {
        if (outs_.count(state_->out_names[i]) && port_in_xml(*this, state_->out_names[i])) {
          setOutput(state_->out_names[i], state_->out_values[i]);
        }
      }
    }
    return state_->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    if (!state_ || !client_) {return;}
    std::lock_guard<std::mutex> lk(state_->mu);
    if (state_->handle && !state_->done) {client_->async_cancel_goal(state_->handle);}
  }

private:
  std::shared_ptr<ExecContext> ctx_;
  std::string ext_, id_;
  std::vector<std::string> ports_;
  std::set<std::string> outs_;
  rclcpp_action::Client<ExecuteNode>::SharedPtr client_;
  std::shared_ptr<GoalState> state_;
};
}  // namespace

int register_extensions(
  BT::BehaviorTreeFactory & f,
  const std::map<std::string, bt_pipeline_msgs::msg::ExtensionManifest> & manifests,
  std::shared_ptr<ExecContext> ctx)
{
  int n = 0;
  for (const auto & [ext, man] : manifests) {
    for (const auto & spec : man.nodes) {
      if (spec.category != "Action" && spec.category != "Condition") {
        ctx->log("unsupported category " + spec.category + " for " + spec.id + " (skipped)");
        continue;
      }
      BT::TreeNodeManifest tm;
      tm.type = BT::NodeType::ACTION;
      tm.registration_ID = spec.id;
      std::vector<std::string> ports;
      std::set<std::string> outs;    // out_<name>: written to the blackboard from the action result
      auto add = [&](const std::string & p) {
          if (p.rfind("out_", 0) == 0) {
            tm.ports.insert(BT::OutputPort<std::string>(p)); outs.insert(p);
          } else {
            tm.ports.insert(BT::InputPort<std::string>(p)); ports.push_back(p);
          }
        };
      for (const auto & p : spec.required_ports) {add(p);}
      for (const auto & p : spec.optional_ports) {add(p);}
      const std::string id = spec.id;
      f.registerBuilder(
        tm, [ctx, ext, id, ports, outs](const std::string & name, const BT::NodeConfig & cfg) {
          return std::make_unique<ExtNode>(name, cfg, ctx, ext, id, ports, outs);
        });
      ++n;
    }
  }
  return n;
}
bool plan_skips_nodes(
  const std::string & xml,
  const std::map<std::string, bt_pipeline_msgs::msg::ExtensionManifest> & manifests,
  const std::set<std::string> & exempt)
{
  for (const auto & [ext, man] : manifests) {
    if (exempt.count(ext)) {continue;}
    for (const auto & spec : man.nodes) {
      const std::string tag = "<" + spec.id;
      for (auto at = xml.find(tag); at != std::string::npos; at = xml.find(tag, at + 1)) {
        const size_t end = at + tag.size();        // "<Speak" must not match "<SpeakLoud"
        if (end >= xml.size() || xml[end] == ' ' || xml[end] == '/' || xml[end] == '>' || xml[end] == '\n') {return true;}
      }
    }
  }
  return false;
}

}  // namespace bt_executor
