#include <gtest/gtest.h>

#include <chrono>

#include <cmath>
#include <queue>
#include <random>
#include <vector>

#include "dstar_lite_planner/core.hpp"

using dstar_lite_planner::Core;
using dstar_lite_planner::kInf;

namespace
{
// Independent oracle: plain Dijkstra with the edge rules written out again (not Core::step).
struct Grid
{
  int w, h;
  std::vector<double> c;   // <0 blocked
  bool blocked(int x, int y) const {return x < 0 || y < 0 || x >= w || y >= h || c[y * w + x] < 0;}
  double edge(int ax, int ay, int bx, int by) const
  {
    if (blocked(ax, ay) || blocked(bx, by)) {return kInf;}
    const int dx = bx - ax, dy = by - ay;
    if (dx != 0 && dy != 0) {
      if (blocked(ax + dx, ay) || blocked(ax, ay + dy)) {return kInf;}
      return std::sqrt(2.0) * (1.0 + c[by * w + bx]);
    }
    return 1.0 + c[by * w + bx];
  }
};

double dijkstra(const Grid & g, int sx, int sy, int tx, int ty)
{
  std::vector<double> d(g.w * g.h, kInf);
  using P = std::pair<double, int>;
  std::priority_queue<P, std::vector<P>, std::greater<P>> q;
  d[sy * g.w + sx] = 0;
  q.push({0, sy * g.w + sx});
  while (!q.empty()) {
    auto [dist, u] = q.top();
    q.pop();
    if (dist > d[u]) {continue;}
    const int x = u % g.w, y = u / g.w;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (!dx && !dy) {continue;}
        const int nx = x + dx, ny = y + dy;
        if (g.blocked(nx, ny)) {continue;}
        const double e = g.edge(x, y, nx, ny);
        if (e < kInf && dist + e < d[ny * g.w + nx]) {
          d[ny * g.w + nx] = dist + e;
          q.push({d[ny * g.w + nx], ny * g.w + nx});
        }
      }
    }
  }
  return d[ty * g.w + tx];
}

// total cost of a returned path under the oracle's edge rules; also checks it is a legal walk
double path_cost(const Grid & g, const std::vector<std::pair<int, int>> & p)
{
  double sum = 0;
  for (size_t i = 1; i < p.size(); ++i) {
    EXPECT_LE(std::abs(p[i].first - p[i - 1].first), 1);
    EXPECT_LE(std::abs(p[i].second - p[i - 1].second), 1);
    const double e = g.edge(p[i - 1].first, p[i - 1].second, p[i].first, p[i].second);
    EXPECT_LT(e, kInf);
    sum += e;
  }
  return sum;
}

Grid random_grid(std::mt19937 & r, int w, int h, double p_block, double p_pen)
{
  Grid g{w, h, std::vector<double>(w * h, 0.0)};
  std::uniform_real_distribution<double> u(0, 1);
  for (auto & v : g.c) {
    const double a = u(r);
    v = a < p_block ? -1.0 : (a < p_block + p_pen ? 3.0 * u(r) : 0.0);
  }
  return g;
}
}  // namespace

TEST(Core, OpenGridStraightAndDiagonal)
{
  Core c(20, 20);
  c.reset(15, 3, 3, 3);
  ASSERT_TRUE(c.compute());
  EXPECT_NEAR(c.cost_to_goal(), 12.0, 1e-9);
  EXPECT_EQ(c.path().size(), 13u);
  Core d(20, 20);
  d.reset(10, 10, 0, 0);
  ASSERT_TRUE(d.compute());
  EXPECT_NEAR(d.cost_to_goal(), 10 * std::sqrt(2.0), 1e-9);
}

TEST(Core, WallWithGapGoesThroughTheGap)
{
  Grid g{21, 21, std::vector<double>(21 * 21, 0.0)};
  for (int y = 0; y < 21; ++y) {if (y != 15) {g.c[y * 21 + 10] = -1;}}
  Core c(21, 21);
  c.set_costs(g.c);
  c.reset(18, 2, 2, 2);
  ASSERT_TRUE(c.compute());
  bool through_gap = false;
  for (auto [x, y] : c.path()) {if (x == 10) {through_gap = (y == 15);}}
  EXPECT_TRUE(through_gap);
  EXPECT_NEAR(path_cost(g, c.path()), dijkstra(g, 2, 2, 18, 2), 1e-9);
}

TEST(Core, UnreachableAndBlockedGoal)
{
  Grid g{10, 10, std::vector<double>(100, 0.0)};
  for (int y = 0; y < 10; ++y) {g.c[y * 10 + 5] = -1;}       // full wall
  Core c(10, 10);
  c.set_costs(g.c);
  c.reset(8, 5, 1, 5);
  EXPECT_FALSE(c.compute());
  EXPECT_TRUE(c.path().empty());
  g.c[5 * 10 + 8] = -1;                                       // goal itself blocked
  Core d(10, 10);
  d.set_costs(g.c);
  d.reset(8, 5, 6, 5);
  EXPECT_FALSE(d.compute());
}

TEST(Core, DiagonalDoesNotCutCorners)
{
  Grid g{3, 3, std::vector<double>(9, 0.0)};
  g.c[0 * 3 + 1] = -1;      // (1,0)
  g.c[1 * 3 + 0] = -1;      // (0,1): both flanks of the diagonal (0,0)->(1,1) are blocked
  Core c(3, 3);
  c.set_costs(g.c);
  c.reset(2, 2, 0, 0);
  EXPECT_FALSE(c.compute());
}

TEST(Core, PenaltyPrefersTheCheaperRoute)
{
  Grid g{15, 5, std::vector<double>(75, 0.0)};
  for (int x = 3; x < 12; ++x) {g.c[2 * 15 + x] = 5.0;}       // expensive straight corridor
  Core c(15, 5);
  c.set_costs(g.c);
  c.reset(14, 2, 0, 2);
  ASSERT_TRUE(c.compute());
  EXPECT_NEAR(c.cost_to_goal(), dijkstra(g, 0, 2, 14, 2), 1e-9);
  bool detours = false;
  for (auto [x, y] : c.path()) {if (y != 2) {detours = true;}}
  EXPECT_TRUE(detours);
}

TEST(Core, MatchesDijkstraOnManyRandomGrids)
{
  std::mt19937 r(1234);
  int solved = 0, unreachable = 0;
  for (int t = 0; t < 300; ++t) {
    Grid g = random_grid(r, 24, 18, 0.28, 0.2);
    const int sx = r() % 24, sy = r() % 18, tx = r() % 24, ty = r() % 18;
    g.c[sy * 24 + sx] = 0;
    g.c[ty * 24 + tx] = 0;
    Core c(24, 18);
    c.set_costs(g.c);
    c.reset(tx, ty, sx, sy);
    const double ref = dijkstra(g, sx, sy, tx, ty);
    EXPECT_EQ(c.compute(), ref < kInf) << "trial " << t;
    if (ref < kInf) {
      EXPECT_NEAR(c.cost_to_goal(), ref, 1e-6) << "trial " << t;
      EXPECT_NEAR(path_cost(g, c.path()), ref, 1e-6) << "trial " << t;
      ++solved;
    } else {
      ++unreachable;
    }
  }
  EXPECT_GT(solved, 100);          // the property is only meaningful if both cases occur
  EXPECT_GT(unreachable, 5);
}

TEST(Core, IncrementalRepairStaysOptimalWhileTheStartMovesAndCellsChange)
{
  std::mt19937 r(99);
  int checks = 0;
  for (int t = 0; t < 60; ++t) {
    Grid g = random_grid(r, 30, 20, 0.15, 0.15);
    int sx = r() % 30, sy = r() % 20;
    const int tx = r() % 30, ty = r() % 20;
    g.c[sy * 30 + sx] = 0;
    g.c[ty * 30 + tx] = 0;
    Core c(30, 20);
    c.set_costs(g.c);
    c.reset(tx, ty, sx, sy);
    for (int round = 0; round < 12; ++round) {
      const double ref = dijkstra(g, sx, sy, tx, ty);
      ASSERT_EQ(c.compute(), ref < kInf) << "trial " << t << " round " << round;
      ++checks;
      if (ref >= kInf) {break;}
      ASSERT_NEAR(c.cost_to_goal(), ref, 1e-6) << "trial " << t << " round " << round;
      ASSERT_NEAR(path_cost(g, c.path()), ref, 1e-6);
      // the robot advances up to 3 cells along the plan, then 1-6 cells change (never the new
      // start or the goal)
      auto p = c.path();
      const size_t adv = std::min<size_t>(p.size() - 1, 1 + r() % 3);
      sx = p[adv].first;
      sy = p[adv].second;
      c.move_start(sx, sy);
      for (int k = 0, n = 1 + r() % 6; k < n; ++k) {
        const int x = r() % 30, y = r() % 20;
        if ((x == sx && y == sy) || (x == tx && y == ty)) {continue;}
        g.c[y * 30 + x] = (r() % 3 == 0) ? 0.0 : ((r() % 2) ? -1.0 : 2.0);
      }
      c.set_costs(g.c);
    }
  }
  EXPECT_GT(checks, 200);
}

TEST(Core, RepairExpandsFarLessThanReplanningFromScratch)
{
  const int n = 120;
  Grid g{n, n, std::vector<double>(n * n, 0.0)};
  Core c(n, n);
  c.set_costs(g.c);
  c.reset(110, 100, 5, 5);
  ASSERT_TRUE(c.compute());
  const long first = c.expansions();
  auto p = c.path();
  const auto mid = p[p.size() / 2];
  for (int d = -2; d <= 2; ++d) {g.c[(mid.second + d) * n + mid.first] = -1;}   // block the path
  c.set_costs(g.c);
  ASSERT_TRUE(c.compute());
  const long repair = c.expansions();
  EXPECT_NEAR(c.cost_to_goal(), dijkstra(g, 5, 5, 110, 100), 1e-6);
  Core fresh(n, n);
  fresh.set_costs(g.c);
  fresh.reset(110, 100, 5, 5);
  ASSERT_TRUE(fresh.compute());
  std::cout << "[ INFO ] expansions: first plan " << first << ", repair " << repair
            << ", fresh replan " << fresh.expansions() << "\n";
  EXPECT_LT(repair, fresh.expansions());
}

TEST(Core, ChangesAwayFromThePathAndMovingAlongItAreNearlyFree)
{
  const int n = 120;
  Grid g{n, n, std::vector<double>(n * n, 0.0)};
  Core c(n, n);
  c.set_costs(g.c);
  c.reset(110, 100, 5, 5);
  ASSERT_TRUE(c.compute());
  const double before = c.cost_to_goal();
  // a handful of cells change in a far corner nobody walks through
  for (int i = 0; i < 8; ++i) {g.c[(100 + i) * n + 10] = -1;}
  c.set_costs(g.c);
  ASSERT_TRUE(c.compute());
  const long unrelated = c.expansions();
  EXPECT_NEAR(c.cost_to_goal(), before, 1e-9);
  // the robot drives 15 cells along the plan; same goal, nothing changed
  auto p = c.path();
  c.move_start(p[15].first, p[15].second);
  c.set_costs(g.c);
  ASSERT_TRUE(c.compute());
  const long moved = c.expansions();
  std::cout << "[ INFO ] expansions: far-away change " << unrelated << ", 15 cells of motion "
            << moved << "\n";
  EXPECT_LT(unrelated, 50);
  EXPECT_LT(moved, 50);
  EXPECT_NEAR(c.cost_to_goal(), dijkstra(g, p[15].first, p[15].second, 110, 100), 1e-6);
}

// Not a pass/fail speed gate (CI machines vary): prints the cost of a fresh search on a sample-map-sized grid so a
// regression or an optimisation is visible in the test log (`colcon test-result`, or run the binary directly).
TEST(Core, FreshSearchTiming)
{
  std::mt19937 r(7);
  const Grid g = random_grid(r, 240, 160, 0.08, 0.15);      // ~ the 12 x 8 m sample map at 5 cm
  const int runs = 20;
  const auto t0 = std::chrono::steady_clock::now();
  long expansions = 0;
  for (int i = 0; i < runs; ++i) {
    Core c(g.w, g.h);
    c.set_costs(g.c);
    c.reset(g.w - 3, g.h - 3, 2, 2);
    c.compute();
    expansions += c.expansions();
  }
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / runs;
  std::printf("[ TIMING ] fresh search on 240x160: %.2f ms (avg of %d, %ld expansions each)\n", ms, runs,
    expansions / runs);
  SUCCEED();
}
