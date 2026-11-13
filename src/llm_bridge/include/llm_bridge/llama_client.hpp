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
  LlamaClient(std::string base_url, double timeout_sec);
  ChatResult chat(const ChatRequest & req) const;

private:
  std::string base_url_;
  double timeout_sec_;
};
}  // namespace llm_bridge
