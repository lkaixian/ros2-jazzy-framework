#include "dstar_lite_planner/core.hpp"

#include <algorithm>

namespace dstar_lite_planner
{
namespace
{
constexpr double kSqrt2 = 1.4142135623730951;
constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
}  // namespace

Core::Core(int width, int height)
: w_(width), h_(height), cost_(static_cast<size_t>(width) * height, 0.0),
  g_(cost_.size(), kInf), rhs_(cost_.size(), kInf), qkey_(cost_.size()), queued_(cost_.size(), 0)
{
}

double Core::step(int a, int b) const
{
  const int ax = a % w_, ay = a / w_, bx = b % w_, by = b / w_;
  if (cost_[a] < 0 || cost_[b] < 0) {return kInf;}
  const int dx = bx - ax, dy = by - ay;
  if (dx != 0 && dy != 0) {                       // diagonal: both flanking cells must be free
    if (cost_[id(ax + dx, ay)] < 0 || cost_[id(ax, ay + dy)] < 0) {return kInf;}
    return kSqrt2 * (1.0 + cost_[b]);
  }
  return 1.0 + cost_[b];
}

double Core::h(int a, int b) const
{
  const double dx = std::abs(a % w_ - b % w_), dy = std::abs(a / w_ - b / w_);
  return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
}

Core::Key Core::key(int s) const
{
  const double m = std::min(g_[s], rhs_[s]);
  return {m + h(id(sx_, sy_), s) + km_, m};
}

bool Core::consistent(int s) const
{
  return g_[s] == rhs_[s] || std::abs(g_[s] - rhs_[s]) < 1e-9;
}

void Core::insert(int s)
{
  qkey_[s] = key(s);
  queued_[s] = 1;
  q_.push({qkey_[s], s});
}

Core::Key Core::top_key()
{
  while (!q_.empty()) {
    const Entry & e = q_.top();
    if (queued_[e.cell] && !(qkey_[e.cell] != e.key)) {return e.key;}
    q_.pop();     // stale: the cell was removed or re-queued with another key
  }
  return {kInf, kInf};
}

void Core::update_vertex(int s)
{
  if (s != id(gx_, gy_)) {
    double best = kInf;
    const int x = s % w_, y = s / w_;
    for (int i = 0; i < 8; ++i) {
      const int nx = x + kDx[i], ny = y + kDy[i];
      if (nx < 0 || ny < 0 || nx >= w_ || ny >= h_) {continue;}
      const int t = id(nx, ny);
      best = std::min(best, step(s, t) + g_[t]);
    }
    rhs_[s] = best;
  }
  queued_[s] = 0;
  if (!consistent(s)) {insert(s);}
}

int Core::set_costs(const std::vector<double> & cost)
{
  if (cost.size() != cost_.size()) {return 0;}
  std::vector<int> changed;
  for (size_t i = 0; i < cost.size(); ++i) {
    if (cost[i] != cost_[i]) {changed.push_back(static_cast<int>(i));}
  }
  cost_ = cost;
  if (active_) {
    // an edge a->b depends on cells a, b and (diagonals) the two flanks: all within 1 cell of
    // the changed one, so repairing its 3x3 neighbourhood covers every affected vertex
    for (int c : changed) {
      const int x = c % w_, y = c / w_;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = x + dx, ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < w_ && ny < h_) {update_vertex(id(nx, ny));}
        }
      }
    }
  }
  return static_cast<int>(changed.size());
}

void Core::reset(int gx, int gy, int sx, int sy)
{
  std::fill(g_.begin(), g_.end(), kInf);
  std::fill(rhs_.begin(), rhs_.end(), kInf);
  std::fill(queued_.begin(), queued_.end(), 0);
  q_ = {};
  gx_ = gx; gy_ = gy; sx_ = lx_ = sx; sy_ = ly_ = sy;
  km_ = 0.0;
  rhs_[id(gx, gy)] = 0.0;
  active_ = true;
  insert(id(gx, gy));
}

void Core::move_start(int sx, int sy)
{
  if (!active_) {return;}
  km_ += h(id(lx_, ly_), id(sx, sy));
  lx_ = sx; ly_ = sy;
  sx_ = sx; sy_ = sy;
}

bool Core::compute()
{
  expansions_ = 0;
  if (!active_) {return false;}
  const int s = id(sx_, sy_);
  while (true) {
    const Key top = top_key();
    if (!(top < key(s)) && consistent(s)) {break;}
    if (q_.empty()) {break;}                       // nothing left to expand: unreachable
    const Entry e = q_.top();
    q_.pop();
    const int u = e.cell;
    queued_[u] = 0;
    const Key k_new = key(u);
    if (e.key < k_new) {insert(u); continue;}       // km grew since it was queued
    ++expansions_;
    const int x = u % w_, y = u / w_;
    if (g_[u] > rhs_[u]) {
      g_[u] = rhs_[u];
    } else {
      g_[u] = kInf;
      update_vertex(u);
    }
    for (int i = 0; i < 8; ++i) {
      const int nx = x + kDx[i], ny = y + kDy[i];
      if (nx >= 0 && ny >= 0 && nx < w_ && ny < h_) {update_vertex(id(nx, ny));}
    }
  }
  return g_[s] < kInf;
}

std::vector<std::pair<int, int>> Core::path() const
{
  std::vector<std::pair<int, int>> out;
  if (!active_ || g_[id(sx_, sy_)] == kInf) {return out;}
  int cur = id(sx_, sy_);
  const int goal = id(gx_, gy_);
  out.emplace_back(cur % w_, cur / w_);
  for (long guard = 0; cur != goal; ++guard) {
    if (guard > static_cast<long>(w_) * h_) {return {};}      // cannot loop on a consistent g
    double best = kInf;
    int next = -1;
    const int x = cur % w_, y = cur / w_;
    for (int i = 0; i < 8; ++i) {
      const int nx = x + kDx[i], ny = y + kDy[i];
      if (nx < 0 || ny < 0 || nx >= w_ || ny >= h_) {continue;}
      const int t = id(nx, ny);
      const double v = step(cur, t) + g_[t];
      if (v < best) {best = v; next = t;}
    }
    if (next < 0) {return {};}
    cur = next;
    out.emplace_back(cur % w_, cur / w_);
  }
  return out;
}
}  // namespace dstar_lite_planner
