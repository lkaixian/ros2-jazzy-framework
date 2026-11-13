#pragma once
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "llm_bridge/llama_client.hpp"

namespace llm_bridge
{
struct FailoverConfig
{
  size_t endpoints = 1;         ///< number of backends, in priority order (0 = preferred)
  int max_failures = 2;         ///< consecutive transport/HTTP failures that open the breaker
  double cooldown_sec = 15.0;   ///< an open endpoint is skipped this long, then probed again
  int json_retries = 1;         ///< extra asks on the SAME endpoint when json_mode text is invalid
};

struct FailoverResult
{
  ChatResult chat;
  int endpoint = -1;               ///< index that produced chat (or -1)
  std::vector<std::string> errors;  ///< one line per failed attempt, for logs
  /// No endpoint answered at all (every one skipped or failed to respond). Distinct from
  /// "answered with garbage", which is a bad reply and not an outage.
  bool all_down = false;
};

/// True if text parses as a JSON object.
bool is_json_object(const std::string & text);

/// Ordered failover with a per-endpoint circuit breaker. Transport-agnostic: the caller supplies
/// the chat function (index -> result) and a monotonic clock in seconds, so it is unit-testable
/// without a network. Thread-safe; chat functions run outside the lock.
class FailoverClient
{
public:
  using ChatFn = std::function<ChatResult(size_t endpoint, const ChatRequest &)>;
  using Clock = std::function<double()>;

  FailoverClient(FailoverConfig cfg, ChatFn fn, Clock clock);

  FailoverResult chat(const ChatRequest & req);
  /// Feed an out-of-band health probe result (success closes the breaker at once).
  void report_health(size_t endpoint, bool ok);
  /// available()[i] is false while endpoint i has failed max_failures times in a row.
  std::vector<bool> available() const;
  bool all_unavailable() const;

private:
  struct State
  {
    int failures = 0;
    double open_until = 0.0;
  };
  bool skipped(size_t i, double now) const;
  void note(size_t i, bool ok);

  FailoverConfig cfg_;
  ChatFn fn_;
  Clock clock_;
  mutable std::mutex mu_;
  std::vector<State> st_;
};
}  // namespace llm_bridge
