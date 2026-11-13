#pragma once
// Incremental D* Lite (Koenig & Likhachev 2002) on an 8-connected grid, no ROS dependency.
// Ported from the 4-connected, binary-grid, rebuild-every-time version in lkaixian/dstarlite-py:
// same backward search from the goal, g/rhs values and (k1,k2) keys, plus what D* Lite is for:
// the search survives between calls (km offset when the start moves, repair of only the vertices
// next to changed cells) and edge costs come from a cost grid instead of 0/1.
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace dstar_lite_planner
{
constexpr double kInf = std::numeric_limits<double>::infinity();

class Core
{
public:
  Core(int width, int height);

  int width() const {return w_;}
  int height() const {return h_;}

  /// Cost per cell, size w*h: < 0 = blocked, otherwise a penalty >= 0 added to every step INTO
  /// the cell (step = base * (1 + penalty), base 1 or sqrt(2)). Returns how many cells changed;
  /// an active search is repaired in place.
  int set_costs(const std::vector<double> & cost);

  /// Start a new search toward (gx,gy) for a robot at (sx,sy). Forgets g/rhs/km.
  void reset(int gx, int gy, int sx, int sy);
  /// The robot moved: keep the search (km += h(old,new), as in the paper).
  void move_start(int sx, int sy);

  bool active() const {return active_;}
  int goal_x() const {return gx_;}
  int goal_y() const {return gy_;}

  /// Expand until the start is consistent. True if a path exists.
  bool compute();
  /// start -> goal cells (empty if none). Valid after compute() returned true.
  std::vector<std::pair<int, int>> path() const;

  double cost_to_goal() const {return active_ ? g_[id(sx_, sy_)] : kInf;}
  long expansions() const {return expansions_;}   // vertices expanded by the last compute()

  /// Cost of the edge a -> b (cell ids); kInf if blocked or a diagonal would cut a corner.
  double step(int a, int b) const;
  int id(int x, int y) const {return y * w_ + x;}

private:
  struct Key
  {
    double k1, k2;
    bool operator<(const Key & o) const {return k1 < o.k1 || (k1 == o.k1 && k2 < o.k2);}
    bool operator!=(const Key & o) const {return k1 != o.k1 || k2 != o.k2;}
  };
  struct Entry
  {
    Key key;
    int cell;
    bool operator<(const Entry & o) const {return o.key < key;}   // min-heap
  };

  double h(int a, int b) const;      // octile distance, never above the true cost
  Key key(int s) const;
  bool consistent(int s) const;
  void insert(int s);
  void update_vertex(int s);
  Key top_key();                     // drops stale entries

  int w_, h_;
  bool active_ = false;
  int gx_ = 0, gy_ = 0, sx_ = 0, sy_ = 0, lx_ = 0, ly_ = 0;   // goal, start, last start
  double km_ = 0.0;
  long expansions_ = 0;
  std::vector<double> cost_, g_, rhs_;
  std::vector<Key> qkey_;            // key a cell is queued with (valid while queued_)
  std::vector<char> queued_;
  std::priority_queue<Entry> q_;
};
}  // namespace dstar_lite_planner
