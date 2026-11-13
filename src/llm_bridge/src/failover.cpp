#include "llm_bridge/failover.hpp"

#include <nlohmann/json.hpp>
#include <utility>

namespace llm_bridge
{
bool is_json_object(const std::string & text)
{
  const auto j = nlohmann::json::parse(text, nullptr, false);
  return !j.is_discarded() && j.is_object();
}

FailoverClient::FailoverClient(FailoverConfig cfg, ChatFn fn, Clock clock)
: cfg_(cfg), fn_(std::move(fn)), clock_(std::move(clock)), st_(cfg.endpoints)
{
}

bool FailoverClient::skipped(size_t i, double now) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return st_[i].failures >= cfg_.max_failures && now < st_[i].open_until;
}

void FailoverClient::note(size_t i, bool ok)
{
  const double now = clock_();
  std::lock_guard<std::mutex> lk(mu_);
  if (ok) {
    st_[i].failures = 0;
    st_[i].open_until = 0.0;
    return;
  }
  ++st_[i].failures;
  if (st_[i].failures >= cfg_.max_failures) {
    // also re-opens at once when a half-open probe after the cooldown fails
    st_[i].open_until = now + cfg_.cooldown_sec;
  }
}

FailoverResult FailoverClient::chat(const ChatRequest & req)
{
  FailoverResult out;
  bool any_answered = false;
  for (size_t i = 0; i < cfg_.endpoints; ++i) {
    const std::string tag = "endpoint " + std::to_string(i);
    if (skipped(i, clock_())) {
      out.errors.push_back(tag + ": skipped (breaker open)");
      continue;
    }
    for (int attempt = 0; attempt <= cfg_.json_retries; ++attempt) {
      ChatResult r = fn_(i, req);
      if (!r.ok) {
        note(i, false);
        out.errors.push_back(tag + ": " + (r.error.empty() ? "failed" : r.error));
        break;  // transport/HTTP failure: go to the next endpoint
      }
      any_answered = true;
      note(i, true);  // it answered, so it is alive even if the text is bad
      if (req.json_mode && !is_json_object(r.text)) {
        out.errors.push_back(tag + ": reply is not a JSON object");
        continue;  // ask the same endpoint again, up to json_retries
      }
      out.chat = std::move(r);
      out.endpoint = static_cast<int>(i);
      return out;
    }
  }
  out.all_down = !any_answered;
  out.chat.ok = false;
  out.chat.error = out.all_down ? "all LLM endpoints unavailable" :
    "LLM replied but never with a valid JSON object";
  return out;
}

void FailoverClient::report_health(size_t endpoint, bool ok)
{
  if (endpoint < st_.size()) {note(endpoint, ok);}
}

std::vector<bool> FailoverClient::available() const
{
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<bool> v;
  for (const auto & s : st_) {v.push_back(s.failures < cfg_.max_failures);}
  return v;
}

bool FailoverClient::all_unavailable() const
{
  for (bool a : available()) {
    if (a) {return false;}
  }
  return true;
}
}  // namespace llm_bridge
