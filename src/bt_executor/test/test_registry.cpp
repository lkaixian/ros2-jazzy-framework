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
  std::vector<std::string> * logs = nullptr, std::set<std::string> exempt = {})
{
  BT::BehaviorTreeFactory f;
  auto ctx = std::make_shared<ExecContext>();
  ctx->dry_run = true;
  ctx->dry_run_exempt = exempt;
  if (logs) {ctx->log = [logs](const std::string & s) {logs->push_back(s);};}
  register_perception(f, cache, 1.0);
  register_extensions(f, manifests(), ctx);
  return f.createTreeFromText(xml).tickWhileRunning();
}
}  // namespace

TEST(Registry, PlanSkipsNodesOnlyForNonExemptExtensions)
{
  const auto m = manifests();
  const std::string xml = tree(R"(<DriveDistance meters="0.5" speed="0.1"/>)");
  EXPECT_TRUE(plan_skips_nodes(xml, m, {}));
  EXPECT_FALSE(plan_skips_nodes(xml, m, {"odom_extension"}));        // exempt: it will really run
  EXPECT_FALSE(plan_skips_nodes(tree(R"(<AlwaysSuccess/>)"), m, {}));    // no extension node at all
  EXPECT_FALSE(plan_skips_nodes(tree(R"(<DriveDistanceX meters="1"/>)"), m, {}));   // a longer name is a different node
}

TEST(Registry, ExemptExtensionIsNotDryRun)
{
  // No ROS node in this test: a real (non dry-run) call has nobody to send the goal to, so it must FAIL
  // instead of the dry-run SUCCESS, and it must not log the "[dry-run]" line.
  std::vector<std::string> logs;
  auto st = run(tree(R"(<DriveDistance meters="0.5" speed="0.1"/>)"),
    std::make_shared<PerceptionCache>(), &logs, {"odom_extension"});
  EXPECT_EQ(st, BT::NodeStatus::FAILURE);
  for (const auto & l : logs) {EXPECT_EQ(l.find("[dry-run]"), std::string::npos);}
}

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

TEST(Registry, ExtractFaceFeaturePicksLargestFaceWithEmbeddingAndWiresOutputs)
{
  auto cache = std::make_shared<PerceptionCache>();
  face_recognition_msgr::msg::FaceArray a;
  auto mk = [](const char * id, float size, bool emb) {
      face_recognition_msgr::msg::Face f;
      f.identity = id; f.confidence = 0.9f; f.similarity = 0.8f;
      f.x_max = size; f.y_max = size;
      if (emb) {f.embedding = {0.1f, 0.2f};}
      return f;
    };
  a.faces = {mk("bob", 10, true), mk("alice", 100, true), mk("carol", 500, false)};
  cache->on_faces(a, Clock::now());
  std::vector<std::string> logs;
  EXPECT_EQ(
    run(tree(R"(<ExtractFaceFeature out_identity="{who}"/><DriveDistance meters="{who}"/>)"),
    cache, &logs), BT::NodeStatus::SUCCESS);
  bool found = false;
  for (const auto & l : logs) {found |= l.find("meters=alice") != std::string::npos;}
  EXPECT_TRUE(found) << "largest face WITH an embedding (alice) must feed the next node";
}

TEST(Registry, ExtractFaceFeatureFailsWithoutEmbeddingOrConfidenceOrFreshness)
{
  auto cache = std::make_shared<PerceptionCache>();
  EXPECT_EQ(run(tree("<ExtractFaceFeature/>"), cache), BT::NodeStatus::FAILURE);   // nothing yet
  face_recognition_msgr::msg::FaceArray a;
  face_recognition_msgr::msg::Face f;
  f.identity = "alice"; f.confidence = 0.4f; f.x_max = 50; f.y_max = 50;
  a.faces = {f};
  cache->on_faces(a, Clock::now());
  EXPECT_EQ(run(tree("<ExtractFaceFeature/>"), cache), BT::NodeStatus::FAILURE);   // no embedding
  a.faces[0].embedding = {1.0f};
  cache->on_faces(a, Clock::now());
  EXPECT_EQ(run(tree("<ExtractFaceFeature/>"), cache), BT::NodeStatus::FAILURE);   // conf 0.4 < 0.5
  EXPECT_EQ(run(tree(R"(<ExtractFaceFeature min_conf="0.3"/>)"), cache), BT::NodeStatus::SUCCESS);
  cache->on_faces(a, Clock::now() - std::chrono::seconds(5));
  EXPECT_EQ(run(tree(R"(<ExtractFaceFeature min_conf="0.3"/>)"), cache), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
