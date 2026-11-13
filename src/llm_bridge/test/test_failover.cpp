#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "llm_bridge/failover.hpp"

using llm_bridge::ChatRequest;
using llm_bridge::ChatResult;
using llm_bridge::FailoverClient;
using llm_bridge::FailoverConfig;

namespace
{
// Scripted backends: per endpoint either "down", a text, and a call counter.
struct Fake
{
  std::map<size_t, std::string> text;  // endpoint -> reply text; missing = connection refused
  std::vector<int> calls = std::vector<int>(4, 0);
  double now = 100.0;

  FailoverClient make(size_t n, int max_failures = 2, double cooldown = 15.0, int json_retries = 1)
  {
    FailoverConfig c;
    c.endpoints = n;
    c.max_failures = max_failures;
    c.cooldown_sec = cooldown;
    c.json_retries = json_retries;
    return FailoverClient(
      c, [this](size_t i, const ChatRequest &) {
        ++calls[i];
        ChatResult r;
        auto it = text.find(i);
        if (it == text.end()) {
          r.error = "connection refused";
        } else {
          r.ok = true;
          r.text = it->second;
        }
        return r;
      }, [this] {return now;});
  }
};

ChatRequest json_req()
{
  ChatRequest r;
  r.json_mode = true;
  return r;
}
}  // namespace

TEST(Failover, PrimaryUsedWhenHealthy)
{
  Fake f;
  f.text = {{0, "{\"a\":1}"}, {1, "{\"b\":2}"}};
  auto c = f.make(2);
  auto r = c.chat(json_req());
  EXPECT_TRUE(r.chat.ok);
  EXPECT_EQ(r.endpoint, 0);
  EXPECT_EQ(f.calls[1], 0);
}

TEST(Failover, FallsBackToSecondaryInTheSameCall)
{
  Fake f;
  f.text = {{1, "{\"b\":2}"}};  // primary refuses
  auto c = f.make(2);
  auto r = c.chat(json_req());
  EXPECT_TRUE(r.chat.ok);
  EXPECT_EQ(r.endpoint, 1);
  EXPECT_FALSE(r.all_down);
  ASSERT_EQ(r.errors.size(), 1u);
  EXPECT_NE(r.errors[0].find("connection refused"), std::string::npos);
}

TEST(Failover, BreakerSkipsDeadPrimaryThenReprobesAfterCooldown)
{
  Fake f;
  f.text = {{1, "{\"b\":2}"}};
  auto c = f.make(2, 2, 15.0);
  c.chat(json_req());
  c.chat(json_req());                 // second consecutive failure opens the breaker
  EXPECT_EQ(f.calls[0], 2);
  c.chat(json_req());
  EXPECT_EQ(f.calls[0], 2);           // skipped: no timeout paid
  f.now += 16.0;
  f.text[0] = "{\"a\":1}";            // primary came back
  auto r = c.chat(json_req());
  EXPECT_EQ(f.calls[0], 3);           // probed once the cooldown passed
  EXPECT_EQ(r.endpoint, 0);           // and preferred again
}

TEST(Failover, HalfOpenProbeFailureReopensImmediately)
{
  Fake f;
  f.text = {{1, "{}"}};
  auto c = f.make(2, 2, 15.0);
  c.chat(json_req());
  c.chat(json_req());
  f.now += 16.0;
  c.chat(json_req());                 // probe fails again
  c.chat(json_req());
  EXPECT_EQ(f.calls[0], 3);           // not hammered: reopened for another cooldown
}

TEST(Failover, BothDownIsAllDownAndThenFastFails)
{
  Fake f;                              // nothing answers
  auto c = f.make(2, 2, 15.0);
  auto r = c.chat(json_req());
  EXPECT_FALSE(r.chat.ok);
  EXPECT_TRUE(r.all_down);
  c.chat(json_req());
  EXPECT_TRUE(c.all_unavailable());
  const int before = f.calls[0] + f.calls[1];
  auto r3 = c.chat(json_req());       // both breakers open: no network calls at all
  EXPECT_TRUE(r3.all_down);
  EXPECT_EQ(f.calls[0] + f.calls[1], before);
}

TEST(Failover, GarbageReplyRetriesThenNextEndpointAndIsNotAnOutage)
{
  Fake f;
  f.text = {{0, "sorry, I cannot"}, {1, "{\"ok\":true}"}};
  auto c = f.make(2, 2, 15.0, 1);
  auto r = c.chat(json_req());
  EXPECT_TRUE(r.chat.ok);
  EXPECT_EQ(r.endpoint, 1);
  EXPECT_EQ(f.calls[0], 2);           // asked again once
  EXPECT_TRUE(c.available()[0]);      // garbage is not an outage: breaker stays closed
}

TEST(Failover, AllGarbageFailsWithoutDeclaringOutage)
{
  Fake f;
  f.text = {{0, "nope"}, {1, "[1,2]"}};  // an array is not an object either
  auto c = f.make(2);
  auto r = c.chat(json_req());
  EXPECT_FALSE(r.chat.ok);
  EXPECT_FALSE(r.all_down);
}

TEST(Failover, NonJsonModeAcceptsAnyText)
{
  Fake f;
  f.text = {{0, "plain words"}};
  auto c = f.make(1);
  ChatRequest r;
  EXPECT_TRUE(c.chat(r).chat.ok);
}

TEST(Failover, HealthReportClosesBreakerAtOnce)
{
  Fake f;
  auto c = f.make(2, 2, 15.0);
  c.chat(json_req());
  c.chat(json_req());
  EXPECT_TRUE(c.all_unavailable());
  c.report_health(1, true);
  EXPECT_FALSE(c.all_unavailable());
  EXPECT_FALSE(c.available()[0]);
  EXPECT_TRUE(c.available()[1]);
  c.report_health(0, false);
  c.report_health(0, false);
  EXPECT_FALSE(c.available()[0]);
}

TEST(Failover, IsJsonObject)
{
  EXPECT_TRUE(llm_bridge::is_json_object("{\"a\":[1]}"));
  EXPECT_FALSE(llm_bridge::is_json_object("[1]"));
  EXPECT_FALSE(llm_bridge::is_json_object(""));
  EXPECT_FALSE(llm_bridge::is_json_object("{bad"));
}
