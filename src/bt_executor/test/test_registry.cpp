#include <gtest/gtest.h>

#include <chrono>

#include "bt_executor/registry.hpp"

using namespace bt_executor;
using bt_pipeline_msgs::msg::BtNodeSpec;
using bt_pipeline_msgs::msg::ExtensionManifest;

namespace
{
const char * kTree = R"(<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M">
  <Sequence>%s</Sequence></BehaviorTree></root>)";

std::string tree(const std::string & body)
{
  char buf[2048];
  snprintf(buf, sizeof(buf), kTree, body.c_str());
  return buf;
}

std::map<std::string, ExtensionManifest> manifests()
{
  ExtensionManifest m;
  m.extension = "odom_extension";
  BtNodeSpec s;
  s.id = "DriveDistance"; s.category = "Action"; s.required_ports = {"meters"};
  s.optional_ports = {"speed"};
  m.nodes.push_back(s);
  BtNodeSpec g;
  g.id = "Hover"; g.category = "Control";   // unsupported category
  m.nodes.push_back(g);
  return {{m.extension, m}};
}

BT::NodeStatus run(const std::string & xml, std::shared_ptr<PerceptionCache> cache,
  std::vector<std::string> * logs = nullptr)
{
  BT::BehaviorTreeFactory f;
  auto ctx = std::make_shared<ExecContext>();
  ctx->dry_run = true;
  if (logs) {ctx->log = [logs](const std::string & s) {logs->push_back(s);};}
  register_perception(f, cache, 1.0);
  register_extensions(f, manifests(), ctx);
  return f.createTreeFromText(xml).tickWhileRunning();
}
}  // namespace

TEST(Registry, DryRunExtensionNodeSucceedsAndLogsPorts)
{
  std::vector<std::string> logs;
  auto st = run(tree(R"(<DriveDistance meters="0.5" speed="0.1"/>)"),
    std::make_shared<PerceptionCache>(), &logs);
  EXPECT_EQ(st, BT::NodeStatus::SUCCESS);
  bool found = false;
  for (const auto & l : logs) {
    found |= l.find("odom_extension/DriveDistance(meters=0.5, speed=0.1)") != std::string::npos;
  }
  EXPECT_TRUE(found);
}

TEST(Registry, UnsupportedCategoryIsSkippedNotRegistered)
{
  BT::BehaviorTreeFactory f;
  auto ctx = std::make_shared<ExecContext>();
  EXPECT_EQ(register_extensions(f, manifests(), ctx), 1);
  EXPECT_THROW(f.createTreeFromText(tree("<Hover/>")), std::exception);
}

TEST(Registry, UnknownPortOnExtensionNodeIsRejected)
{
  BT::BehaviorTreeFactory f;
  register_extensions(f, manifests(), std::make_shared<ExecContext>());
  EXPECT_THROW(f.createTreeFromText(tree(R"(<DriveDistance meters="1" bogus="2"/>)")).tickOnce(),
    std::exception);
}

TEST(Registry, DetectObjectUsesLabelConfidenceAndFreshness)
{
  auto cache = std::make_shared<PerceptionCache>();
  object_detection_msgr::msg::ObjectArray a;
  object_detection_msgr::msg::Object o;
  o.label = "cup"; o.confidence = 0.8f; o.x_min = 1; o.y_min = 2; o.x_max = 3; o.y_max = 4;
  a.objects.push_back(o);
  cache->on_objects(a, Clock::now());
  EXPECT_EQ(run(tree(R"(<DetectObject label="cup"/>)"), cache), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(run(tree(R"(<DetectObject label="bottle"/>)"), cache), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run(tree(R"(<DetectObject label="cup" min_conf="0.9"/>)"), cache),
    BT::NodeStatus::FAILURE);
  cache->on_objects(a, Clock::now() - std::chrono::seconds(5));   // stale
  EXPECT_EQ(run(tree(R"(<DetectObject label="cup"/>)"), cache), BT::NodeStatus::FAILURE);
}

TEST(Registry, NothingReceivedMeansFailure)
{
  auto cache = std::make_shared<PerceptionCache>();
  EXPECT_EQ(run(tree("<DetectPerson/>"), cache), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run(tree("<RecognizeFace/>"), cache), BT::NodeStatus::FAILURE);
}

TEST(Registry, RecognizeFaceIdentityAndSimilarity)
{
  auto cache = std::make_shared<PerceptionCache>();
  face_recognition_msgr::msg::FaceArray a;
  face_recognition_msgr::msg::Face f;
  f.identity = "alice"; f.similarity = 0.7f;
  a.faces.push_back(f);
  cache->on_faces(a, Clock::now());
  EXPECT_EQ(run(tree(R"(<RecognizeFace identity="alice"/>)"), cache), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(run(tree(R"(<RecognizeFace identity="bob"/>)"), cache), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run(tree(R"(<RecognizeFace/>)"), cache), BT::NodeStatus::SUCCESS);   // any known
  EXPECT_EQ(run(tree(R"(<RecognizeFace identity="alice" min_similarity="0.9"/>)"), cache),
    BT::NodeStatus::FAILURE);
}

TEST(Registry, FallbackRecoversFromFailedCondition)
{
  auto cache = std::make_shared<PerceptionCache>();
  auto st = run(tree(R"(<Fallback><DetectPerson/><DriveDistance meters="0.2"/></Fallback>)"), cache);
  EXPECT_EQ(st, BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
