#include "llm_bridge/llama_client.hpp"

#include <curl/curl.h>

#include <mutex>
#include <nlohmann/json.hpp>

namespace llm_bridge
{
namespace
{
size_t write_cb(char * ptr, size_t size, size_t nmemb, void * userdata)
{
  static_cast<std::string *>(userdata)->append(ptr, size * nmemb);
  return size * nmemb;
}

void curl_init_once()
{
  static std::once_flag flag;
  std::call_once(flag, [] {curl_global_init(CURL_GLOBAL_DEFAULT);});
}
}  // namespace

std::string build_body(const ChatRequest & req)
{
  nlohmann::ordered_json j;    // ordered: the schema's property order (macro before args) shapes the grammar
  j["model"] = req.model;
  j["temperature"] = req.temperature;
  j["max_tokens"] = req.max_tokens;
  j["stream"] = false;
  if (req.repeat_penalty > 0.0) {j["repeat_penalty"] = req.repeat_penalty;}
  j["messages"] = nlohmann::json::array();
  if (!req.system.empty()) {
    j["messages"].push_back({{"role", "system"}, {"content", req.system}});
  }
  j["messages"].push_back({{"role", "user"}, {"content", req.prompt}});
  if (req.json_mode) {
    auto schema = nlohmann::ordered_json::parse(req.schema, nullptr, false);   // constrained decoding when valid
    if (!req.schema.empty() && !schema.is_discarded()) {
      j["response_format"] = {{"type", "json_schema"},
        {"json_schema", {{"name", "task_spec"}, {"schema", schema}}}};
    } else {
      j["response_format"] = {{"type", "json_object"}};
    }
  }
  return j.dump();
}

ChatResult parse_response(const std::string & body)
{
  ChatResult r;
  try {
    auto j = nlohmann::json::parse(body);
    if (j.contains("error")) {
      r.error = j["error"].is_object() ? j["error"].value("message", j["error"].dump()) :
        j["error"].dump();
      return r;
    }
    r.text = j.at("choices").at(0).at("message").at("content").get<std::string>();
    r.ok = true;
  } catch (const std::exception & e) {
    r.error = std::string("bad response: ") + e.what();
  }
  return r;
}

LlamaClient::LlamaClient(
  std::string base_url, double timeout_sec, double connect_timeout_sec, std::string api_key)
: base_url_(std::move(base_url)), timeout_sec_(timeout_sec),
  connect_timeout_sec_(connect_timeout_sec), api_key_(std::move(api_key))
{
  while (!base_url_.empty() && base_url_.back() == '/') {base_url_.pop_back();}
  curl_init_once();
}

LlamaClient::~LlamaClient()
{
  if (chat_h_) {curl_easy_cleanup(static_cast<CURL *>(chat_h_));}
  if (health_h_) {curl_easy_cleanup(static_cast<CURL *>(health_h_));}
}

namespace
{
// Reuse the handle (its live TCP connection survives curl_easy_reset) and keep the socket alive,
// so an idle link that silently died is noticed instead of hanging until the request timeout.
CURL * prepare(void *& slot)
{
  if (!slot) {slot = curl_easy_init();}
  CURL * c = static_cast<CURL *>(slot);
  if (c) {
    curl_easy_reset(c);
    curl_easy_setopt(c, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(c, CURLOPT_TCP_NODELAY, 1L);
    curl_easy_setopt(c, CURLOPT_TCP_KEEPALIVE, 1L);
    curl_easy_setopt(c, CURLOPT_TCP_KEEPIDLE, 30L);
    curl_easy_setopt(c, CURLOPT_TCP_KEEPINTVL, 10L);
  }
  return c;
}
}  // namespace

ChatResult LlamaClient::chat(const ChatRequest & req) const
{
  ChatResult r;
  std::lock_guard<std::mutex> lk(chat_mu_);
  CURL * curl = prepare(chat_h_);
  if (!curl) {
    r.error = "curl_easy_init failed";
    return r;
  }
  const std::string url = base_url_ + "/v1/chat/completions";
  const std::string body = build_body(req);
  std::string response;
  curl_slist * headers = curl_slist_append(nullptr, "Content-Type: application/json");
  if (!api_key_.empty()) {
    const std::string auth = "Authorization: Bearer " + api_key_;
    headers = curl_slist_append(headers, auth.c_str());   // never logged
  }
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_sec_ * 1000));
  if (connect_timeout_sec_ > 0) {
    curl_easy_setopt(
      curl, CURLOPT_CONNECTTIMEOUT_MS, static_cast<long>(connect_timeout_sec_ * 1000));
  }
  const CURLcode rc = curl_easy_perform(curl);
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  if (rc != CURLE_OK) {
    r.error = std::string("llama-server request failed: ") + curl_easy_strerror(rc) +
      " (" + url + ")";
    return r;
  }
  r = parse_response(response);
  if (!r.ok && r.error.empty()) {r.error = "HTTP " + std::to_string(status);}
  return r;
}

bool LlamaClient::healthy(double timeout_sec) const
{
  std::lock_guard<std::mutex> lk(health_mu_);
  CURL * curl = prepare(health_h_);
  if (!curl) {return false;}
  const std::string url = base_url_ + "/health";
  std::string sink;
  curl_slist * headers = nullptr;
  if (!api_key_.empty()) {
    const std::string auth = "Authorization: Bearer " + api_key_;
    headers = curl_slist_append(headers, auth.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  }
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &sink);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_sec * 1000));
  long status = 0;
  const CURLcode rc = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  if (rc != CURLE_OK) {return false;}
  // 200 = ready. 503 = llama-server still loading its model. 404/405/501 = reachable but the
  // server has no /health (other OpenAI-compatible servers, test doubles): do not call it down.
  return status == 200 || status == 404 || status == 405 || status == 501;
}
}  // namespace llm_bridge
