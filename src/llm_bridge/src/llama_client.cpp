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
  nlohmann::json j;
  j["model"] = req.model;
  j["temperature"] = req.temperature;
  j["max_tokens"] = req.max_tokens;
  j["stream"] = false;
  j["messages"] = nlohmann::json::array();
  if (!req.system.empty()) {
    j["messages"].push_back({{"role", "system"}, {"content", req.system}});
  }
  j["messages"].push_back({{"role", "user"}, {"content", req.prompt}});
  if (req.json_mode) {
    j["response_format"] = {{"type", "json_object"}};
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

LlamaClient::LlamaClient(std::string base_url, double timeout_sec)
: base_url_(std::move(base_url)), timeout_sec_(timeout_sec)
{
  while (!base_url_.empty() && base_url_.back() == '/') {base_url_.pop_back();}
  curl_init_once();
}

ChatResult LlamaClient::chat(const ChatRequest & req) const
{
  ChatResult r;
  CURL * curl = curl_easy_init();
  if (!curl) {
    r.error = "curl_easy_init failed";
    return r;
  }
  const std::string url = base_url_ + "/v1/chat/completions";
  const std::string body = build_body(req);
  std::string response;
  curl_slist * headers = curl_slist_append(nullptr, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_sec_ * 1000));
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  const CURLcode rc = curl_easy_perform(curl);
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  if (rc != CURLE_OK) {
    r.error = std::string("llama-server request failed: ") + curl_easy_strerror(rc) +
      " (" + url + ")";
    return r;
  }
  r = parse_response(response);
  if (!r.ok && r.error.empty()) {r.error = "HTTP " + std::to_string(status);}
  return r;
}
}  // namespace llm_bridge
