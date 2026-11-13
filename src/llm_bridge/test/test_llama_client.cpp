#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "llm_bridge/llama_client.hpp"

TEST(LlamaClient, BuildBody)
{
  llm_bridge::ChatRequest r;
  r.system = "sys";
  r.prompt = "hi";
  r.json_mode = true;
  auto j = nlohmann::json::parse(llm_bridge::build_body(r));
  EXPECT_EQ(j["messages"].size(), 2u);
  EXPECT_EQ(j["messages"][0]["role"], "system");
  EXPECT_EQ(j["response_format"]["type"], "json_object");
  EXPECT_FALSE(j["stream"].get<bool>());
}

TEST(LlamaClient, ParseOk)
{
  auto r = llm_bridge::parse_response(
    R"({"choices":[{"message":{"role":"assistant","content":"{\"a\":1}"}}]})");
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.text, "{\"a\":1}");
}

TEST(LlamaClient, ParseErrors)
{
  EXPECT_FALSE(llm_bridge::parse_response("not json").ok);
  auto r = llm_bridge::parse_response(R"({"error":{"message":"boom"}})");
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.error, "boom");
}

TEST(LlamaClient, UnreachableServerReportsError)
{
  llm_bridge::LlamaClient c("http://127.0.0.1:1", 1.0);
  auto r = c.chat({});
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.error.empty());
}
