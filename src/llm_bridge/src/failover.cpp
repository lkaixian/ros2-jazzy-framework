#include "llm_bridge/failover.hpp"

#include <algorithm>
#include <nlohmann/json.hpp>

namespace llm_bridge
{
bool is_json_object(const std::string & text)
{
  const auto j = nlohmann::json::parse(text, nullptr, false);
  return !j.is_discarded() && j.is_object();
}

FailoverClient::FailoverClient(FailoverConfig cfg, ChatFn fn, Clock clock)
: cfg_(cfg), fn_(std::move(fn)), clock_(std::move(clock))
{
}

std::pair<std::vector<std::string>, std::vector<std::string>> FailoverClient::set_endpoints(
  std::vector<Endpoint> eps, bool probation_new)
{
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<State> next;
  std::vector<std::string> joined, left;
  for (const auto & e : eps) {
    if (e.url.empty() ||
      std::any_of(next.begin(), next.end(), [&](const State & s) {return s.url == e.url;}))
    {
      continue;  // ignore blanks and duplicates
    }
    State s;
    auto old = std::find_if(st_.begin(), st_.end(), [&](const State & o) {return o.url == e.url;});
    if (old != st_.end()) {
      s = *old;
    } else {
      joined.push_back(e.url);
      s.probation = probation_new;
    }
    s.url = e.url;
    s.priority = e.priority;
    next.push_back(s);
  }
  for (const auto & o : st_) {
    if (std::none_of(next.begin(), next.end(), [&](const State & s) {return s.url == o.url;})) {
      left.push_back(o.url);
    }
  }
  std::stable_sort(
    next.begin(), next.end(), [](const State & a, const State & b) {return a.priority < b.priority;});
  st_ = std::move(next);
  return {joined, left};
}

bool FailoverClient::skipped(const std::string & url, double now) const
{
  std::lock_guard<std::mutex> lk(mu_);
  for (const auto & s : st_) {
    if (s.url == url) {
      return s.probation || (s.failures >= cfg_.max_failures && now < s.open_until);
    }
  }
  return true;  // removed while we were iterating: do not call it
}

void FailoverClient::note(const std::string & url, bool ok)
{
  const double now = clock_();
  std::lock_guard<std::mutex> lk(mu_);
  for (auto & s : st_) {
    if (s.url != url) {continue;}
    if (ok) {
      s.failures = 0;
      s.open_until = 0.0;
      s.probation = false;
    } else {
      ++s.failures;
      // also re-opens at once when a half-open probe after the cooldown fails
      if (s.failures >= cfg_.max_failures) {s.open_until = now + cfg_.cooldown_sec;}
    }
    return;
  }
}

FailoverResult FailoverClient::chat(const ChatRequest & req)
{
  std::vector<std::string> order;
  {
    std::lock_guard<std::mutex> lk(mu_);
    for (const auto & s : st_) {order.push_back(s.url);}
  }
  FailoverResult out;
  bool any_answered = false;
  for (size_t i = 0; i < order.size(); ++i) {
    const std::string & url = order[i];
    const std::string tag = "endpoint " + url;
    if (skipped(url, clock_())) {
      out.errors.push_back(tag + ": skipped (breaker open or removed)");
      continue;
    }
    for (int attempt = 0; attempt <= cfg_.json_retries; ++attempt) {
      ChatResult r = fn_(url, req);
      if (!r.ok) {
        note(url, false);
        out.errors.push_back(tag + ": " + (r.error.empty() ? "failed" : r.error));
        break;  // transport/HTTP failure: go to the next endpoint
      }
      any_answered = true;
      note(url, true);  // it answered, so it is alive even if the text is bad
      if (req.json_mode && !is_json_object(r.text)) {
        out.errors.push_back(tag + ": reply is not a JSON object");
        continue;  // ask the same endpoint again, up to json_retries
      }
      out.chat = std::move(r);
      out.endpoint = url;
      out.rank = static_cast<int>(i);
      return out;
    }
  }
  out.all_down = !any_answered;
  out.chat.ok = false;
  out.chat.error = out.all_down ? "all LLM endpoints unavailable" :
    "LLM replied but never with a valid JSON object";
  return out;
}

void FailoverClient::report_health(const std::string & url, bool ok)
{
  note(url, ok);
}

std::vector<std::pair<std::string, bool>> FailoverClient::status() const
{
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<std::pair<std::string, bool>> v;
  for (const auto & s : st_) {v.emplace_back(s.url, !s.probation && s.failures < cfg_.max_failures);}
  return v;
}

bool FailoverClient::all_unavailable() const
{
  for (const auto & kv : status()) {
    if (kv.second) {return false;}
  }
  return true;
}
}  // namespace llm_bridge
