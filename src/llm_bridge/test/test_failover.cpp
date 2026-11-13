#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "llm_bridge/failover.hpp"

using llm_bridge::ChatRequest;
using llm_bridge::ChatResult;
using llm_bridge::FailoverClient;
using llm_bridge::FailoverConfig;

namespace
{
// Scripted backends keyed by url: a reply text, or absent = connection refused.
struct Fake
{
  std::map<std::string, std::string> text;
  std::map<std::string, int> calls;
  double now = 100.0;

  std::unique_ptr<FailoverClient> owned;

  FailoverClient & make(
    std::vector<std::string> urls, int max_failures = 2, double cooldown = 15.0,
    int json_retries = 1)
  {
    FailoverConfig c;
    c.max_failures = max_failures;
    c.cooldown_sec = cooldown;
    c.json_retries = json_retries;
    owned = std::make_unique<FailoverClient>(
      c, [this](const std::string & u, const ChatRequest &) {
        ++calls[u];
        ChatResult r;
        auto it = text.find(u);
        if (it == text.end()) {
          r.error = "connection refused";
        } else {
          r.ok = true;
          r.text = it->second;
        }
        return r;
      }, [this] {return now;});
    std::vector<llm_bridge::Endpoint> eps;
    int p = 0;
    for (const auto & u : urls) {eps.push_back({u, p++});}
    owned->set_endpoints(eps);
    return *owned;
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
  f.text = {{"A", "{\"a\":1}"}, {"B", "{\"b\":2}"}};
  auto & c = f.make({"A", "B"});
  auto r = c.chat(json_req());
  EXPECT_TRUE(r.chat.ok);
  EXPECT_EQ(r.endpoint, "A");
  EXPECT_EQ(r.rank, 0);
  EXPECT_EQ(f.calls["B"], 0);
}

TEST(Failover, FallsBackToSecondaryInTheSameCall)
{
  Fake f;
  f.text = {{"B", "{\"b\":2}"}};  // A refuses
  auto & c = f.make({"A", "B"});
  auto r = c.chat(json_req());
  EXPECT_TRUE(r.chat.ok);
  EXPECT_EQ(r.endpoint, "B");
  EXPECT_EQ(r.rank, 1);
  EXPECT_FALSE(r.all_down);
  ASSERT_EQ(r.errors.size(), 1u);
  EXPECT_NE(r.errors[0].find("connection refused"), std::string::npos);
}

TEST(Failover, BreakerSkipsDeadPrimaryThenReprobesAfterCooldown)
{
  Fake f;
  f.text = {{"B", "{\"b\":2}"}};
  auto & c = f.make({"A", "B"}, 2, 15.0);
  c.chat(json_req());
  c.chat(json_req());                 // second consecutive failure opens the breaker
  EXPECT_EQ(f.calls["A"], 2);
  c.chat(json_req());
  EXPECT_EQ(f.calls["A"], 2);         // skipped: no timeout paid
  f.now += 16.0;
  f.text["A"] = "{\"a\":1}";          // primary came back
  auto r = c.chat(json_req());
  EXPECT_EQ(f.calls["A"], 3);         // probed once the cooldown passed
  EXPECT_EQ(r.endpoint, "A");         // and preferred again
}

TEST(Failover, HalfOpenProbeFailureReopensImmediately)
{
  Fake f;
  f.text = {{"B", "{}"}};
  auto & c = f.make({"A", "B"}, 2, 15.0);
  c.chat(json_req());
  c.chat(json_req());
  f.now += 16.0;
  c.chat(json_req());                 // probe fails again
  c.chat(json_req());
  EXPECT_EQ(f.calls["A"], 3);         // not hammered: reopened for another cooldown
}

TEST(Failover, BothDownIsAllDownAndThenFastFails)
{
  Fake f;                              // nothing answers
  auto & c = f.make({"A", "B"}, 2, 15.0);
  auto r = c.chat(json_req());
  EXPECT_FALSE(r.chat.ok);
  EXPECT_TRUE(r.all_down);
  c.chat(json_req());
  EXPECT_TRUE(c.all_unavailable());
  const int before = f.calls["A"] + f.calls["B"];
  auto r3 = c.chat(json_req());       // both breakers open: no network calls at all
  EXPECT_TRUE(r3.all_down);
  EXPECT_EQ(f.calls["A"] + f.calls["B"], before);
}

TEST(Failover, GarbageReplyRetriesThenNextEndpointAndIsNotAnOutage)
{
  Fake f;
  f.text = {{"A", "sorry, I cannot"}, {"B", "{\"ok\":true}"}};
  auto & c = f.make({"A", "B"}, 2, 15.0, 1);
  auto r = c.chat(json_req());
  EXPECT_TRUE(r.chat.ok);
  EXPECT_EQ(r.endpoint, "B");
  EXPECT_EQ(f.calls["A"], 2);         // asked again once
  EXPECT_TRUE(c.status()[0].second);  // garbage is not an outage: breaker stays closed
}

TEST(Failover, AllGarbageFailsWithoutDeclaringOutage)
{
  Fake f;
  f.text = {{"A", "nope"}, {"B", "[1,2]"}};  // an array is not an object either
  auto & c = f.make({"A", "B"});
  auto r = c.chat(json_req());
  EXPECT_FALSE(r.chat.ok);
  EXPECT_FALSE(r.all_down);
}

TEST(Failover, NonJsonModeAcceptsAnyText)
{
  Fake f;
  f.text = {{"A", "plain words"}};
  auto & c = f.make({"A"});
  ChatRequest r;
  EXPECT_TRUE(c.chat(r).chat.ok);
}

TEST(Failover, HealthReportClosesBreakerAtOnce)
{
  Fake f;
  auto & c = f.make({"A", "B"}, 2, 15.0);
  c.chat(json_req());
  c.chat(json_req());
  EXPECT_TRUE(c.all_unavailable());
  c.report_health("B", true);
  EXPECT_FALSE(c.all_unavailable());
  EXPECT_FALSE(c.status()[0].second);
  EXPECT_TRUE(c.status()[1].second);
  c.report_health("A", false);
  c.report_health("A", false);
  EXPECT_FALSE(c.status()[0].second);
}

TEST(Failover, IsJsonObject)
{
  EXPECT_TRUE(llm_bridge::is_json_object("{\"a\":[1]}"));
  EXPECT_FALSE(llm_bridge::is_json_object("[1]"));
  EXPECT_FALSE(llm_bridge::is_json_object(""));
  EXPECT_FALSE(llm_bridge::is_json_object("{bad"));
}

// ---- hosts joining and leaving -------------------------------------------------------------

TEST(FailoverDynamic, NoEndpointsIsAnOutage)
{
  Fake f;
  auto & c = f.make({});
  EXPECT_TRUE(c.all_unavailable());
  auto r = c.chat(json_req());
  EXPECT_TRUE(r.all_down);
  EXPECT_EQ(r.errors.size(), 0u);
}

TEST(FailoverDynamic, JoinedHostWithBetterPriorityIsPreferred)
{
  Fake f;
  f.text = {{"small", "{\"s\":1}"}, {"big", "{\"b\":1}"}};
  auto & c = f.make({"small"});
  EXPECT_EQ(c.chat(json_req()).endpoint, "small");
  auto delta = c.set_endpoints({{"small", 100}, {"big", 10}});   // the strong PC joins
  EXPECT_EQ(delta.first, std::vector<std::string>{"big"});
  EXPECT_TRUE(delta.second.empty());
  auto r = c.chat(json_req());
  EXPECT_EQ(r.endpoint, "big");
  EXPECT_EQ(r.rank, 0);
}

TEST(FailoverDynamic, LeftHostIsNeverCalledAgain)
{
  Fake f;
  f.text = {{"small", "{\"s\":1}"}, {"big", "{\"b\":1}"}};
  auto & c = f.make({"big", "small"});
  c.chat(json_req());
  auto delta = c.set_endpoints({{"small", 100}});                // the strong PC disconnects
  EXPECT_EQ(delta.second, std::vector<std::string>{"big"});
  const int before = f.calls["big"];
  auto r = c.chat(json_req());
  EXPECT_EQ(r.endpoint, "small");
  EXPECT_EQ(f.calls["big"], before);
  EXPECT_TRUE(r.errors.empty());                                  // no timeout paid for it
}

TEST(FailoverDynamic, StayingEndpointKeepsItsBreakerState)
{
  Fake f;
  f.text = {{"B", "{}"}};
  auto & c = f.make({"A", "B"}, 2, 15.0);
  c.chat(json_req());
  c.chat(json_req());                                             // A's breaker is open
  c.set_endpoints({{"A", 0}, {"B", 1}, {"C", 2}});                // an unrelated host joins
  const int before = f.calls["A"];
  c.chat(json_req());
  EXPECT_EQ(f.calls["A"], before);                                // still skipped
}

TEST(FailoverDynamic, RejoiningHostStartsFresh)
{
  Fake f;
  f.text = {{"B", "{}"}};
  auto & c = f.make({"A", "B"}, 2, 15.0);
  c.chat(json_req());
  c.chat(json_req());                                             // A broken
  c.set_endpoints({{"B", 1}});                                    // A leaves
  f.text["A"] = "{\"a\":1}";
  c.set_endpoints({{"A", 0}, {"B", 1}});                          // A reboots and rejoins
  EXPECT_EQ(c.chat(json_req()).endpoint, "A");                    // no stale cooldown
}

TEST(FailoverDynamic, DuplicatesAndBlanksIgnoredOrderIsStable)
{
  Fake f;
  auto & c = f.make({});
  c.set_endpoints({{"x", 5}, {"", 0}, {"y", 5}, {"x", 1}, {"z", 1}});
  auto st = c.status();
  ASSERT_EQ(st.size(), 3u);
  EXPECT_EQ(st[0].first, "z");   // lowest priority number first
  EXPECT_EQ(st[1].first, "x");   // first x wins (priority 5); equal priority keeps given order
  EXPECT_EQ(st[2].first, "y");
}

TEST(FailoverDynamic, JoinedHostIsOnProbationUntilHealthy)
{
  Fake f;
  f.text = {{"small", "{\"s\":1}"}, {"big", "{\"b\":1}"}};
  auto & c = f.make({"small"});
  c.set_endpoints({{"small", 100}, {"big", 10}}, true);          // big joined, model still loading
  EXPECT_FALSE(c.status()[0].second);                              // big counts as unavailable
  auto r = c.chat(json_req());
  EXPECT_EQ(r.endpoint, "small");
  EXPECT_EQ(f.calls["big"], 0);                                    // no failed attempt paid
  c.report_health("big", false);                                   // /health still 503
  EXPECT_EQ(c.chat(json_req()).endpoint, "small");
  c.report_health("big", true);                                    // model loaded
  EXPECT_EQ(c.chat(json_req()).endpoint, "big");
}

TEST(FailoverDynamic, OnlyProbationEndpointsMeansOutage)
{
  Fake f;
  f.text = {{"big", "{}"}};
  auto & c = f.make({});
  c.set_endpoints({{"big", 10}}, true);
  EXPECT_TRUE(c.all_unavailable());
  EXPECT_TRUE(c.chat(json_req()).all_down);
}
