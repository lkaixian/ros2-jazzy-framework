#pragma once
#include <string>

namespace llm_bridge
{
struct ChatRequest
{
  std::string system;
  std::string prompt;
  std::string model = "default";
  double temperature = 0.1;
  int max_tokens = 512;
  bool json_mode = false;
};

struct ChatResult
{
  bool ok = false;
  std::string text;
  std::string error;
};

/// Build the /v1/chat/completions request body (exposed for testing).
std::string build_body(const ChatRequest & req);
/// Extract choices[0].message.content from a response body.
ChatResult parse_response(const std::string & body);

/// Blocking client for llama.cpp's llama-server. Thread-safe (one handle per call).
class LlamaClient
{
public:
  /// connect_timeout_sec <= 0 leaves curl's default (minutes): set it so a dead host fails fast.
  LlamaClient(std::string base_url, double timeout_sec, double connect_timeout_sec = 0.0);
  ChatResult chat(const ChatRequest & req) const;
  /// GET /health within timeout_sec: true for 200, and for 404/405/501 (reachable, no /health);
  /// false for 503 (model still loading), other errors and connection failures.
  bool healthy(double timeout_sec) const;

private:
  std::string base_url_;
  double timeout_sec_;
  double connect_timeout_sec_;
};
}  // namespace llm_bridge
