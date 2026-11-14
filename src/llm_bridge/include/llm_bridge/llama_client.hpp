#pragma once
#include <mutex>
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
  double repeat_penalty = 0.0;   // > 0 is sent as repeat_penalty
  bool json_mode = false;
  std::string schema;     // JSON schema (text) the reply must follow; empty = any JSON object
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

/// Blocking client for llama.cpp's llama-server over HTTP/TCP. Thread-safe.
/// One persistent curl handle for chat and one for /health, so the TCP connection is reused
/// (no handshake per request) and kept alive; calls on the same handle are serialized.
class LlamaClient
{
public:
  /// connect_timeout_sec <= 0 leaves curl's default (minutes): set it so a dead host fails fast.
  /// api_key: sent as `Authorization: Bearer <key>` when not empty (llama-server --api-key).
  LlamaClient(
    std::string base_url, double timeout_sec, double connect_timeout_sec = 0.0,
    std::string api_key = "");
  ~LlamaClient();
  LlamaClient(const LlamaClient &) = delete;
  LlamaClient & operator=(const LlamaClient &) = delete;
  ChatResult chat(const ChatRequest & req) const;
  /// GET /health within timeout_sec: true for 200, and for 404/405/501 (reachable, no /health);
  /// false for 503 (model still loading), other errors and connection failures.
  bool healthy(double timeout_sec) const;

private:
  std::string base_url_;
  double timeout_sec_;
  double connect_timeout_sec_;
  std::string api_key_;
  mutable void * chat_h_ = nullptr;     // CURL*, created on first use
  mutable void * health_h_ = nullptr;
  mutable std::mutex chat_mu_, health_mu_;
};
}  // namespace llm_bridge
