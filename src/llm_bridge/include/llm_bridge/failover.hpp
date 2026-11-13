#pragma once
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "llm_bridge/llama_client.hpp"

namespace llm_bridge
{
struct FailoverConfig
{
  int max_failures = 2;         ///< consecutive transport/HTTP failures that open the breaker
  double cooldown_sec = 15.0;   ///< an open endpoint is skipped this long, then probed again
  int json_retries = 1;         ///< extra asks on the SAME endpoint when json_mode text is invalid
};

struct Endpoint
{
  std::string url;
  int priority = 0;  ///< lower = tried first; equal priorities keep their given order
};

struct FailoverResult
{
  ChatResult chat;
  std::string endpoint;             ///< url that produced chat ("" if none)
  int rank = -1;                    ///< its position in the priority order (0 = preferred)
  std::vector<std::string> errors;  ///< one line per failed attempt, for logs
  /// No endpoint answered at all (none configured, or every one skipped / failed to respond).
  /// Distinct from "answered with garbage", which is a bad reply and not an outage.
  bool all_down = false;
};

/// True if text parses as a JSON object.
bool is_json_object(const std::string & text);

/// Priority-ordered failover over a changing set of endpoints, with a per-endpoint circuit
/// breaker. Transport-agnostic: the caller supplies the chat function (url -> result) and a
/// monotonic clock in seconds, so it is unit-testable without a network. Thread-safe; chat
/// functions run outside the lock.
class FailoverClient
{
public:
  using ChatFn = std::function<ChatResult(const std::string & url, const ChatRequest &)>;
  using Clock = std::function<double()>;

  FailoverClient(FailoverConfig cfg, ChatFn fn, Clock clock);

  /// Replace the endpoint set (a host joined or left). Breaker state of urls that stay is kept;
  /// a url that left and comes back starts fresh. Returns {joined, left} urls.
  /// probation_new: a joined url is not used (and counts as unavailable) until report_health(url,
  /// true): a server that just appeared is usually still loading its model.
  std::pair<std::vector<std::string>, std::vector<std::string>> set_endpoints(
    std::vector<Endpoint> eps, bool probation_new = false);

  FailoverResult chat(const ChatRequest & req);
  /// Feed an out-of-band health probe result (success closes the breaker at once).
  void report_health(const std::string & url, bool ok);
  /// Endpoints in priority order with availability (false while max_failures in a row failed).
  std::vector<std::pair<std::string, bool>> status() const;
  /// True when there is no endpoint, or none is available.
  bool all_unavailable() const;

private:
  struct State
  {
    std::string url;
    int priority = 0;
    int failures = 0;
    double open_until = 0.0;
    bool probation = false;
  };
  bool skipped(const std::string & url, double now) const;
  void note(const std::string & url, bool ok);

  FailoverConfig cfg_;
  ChatFn fn_;
  Clock clock_;
  mutable std::mutex mu_;
  std::vector<State> st_;  // kept sorted by priority (stable)
};
}  // namespace llm_bridge
