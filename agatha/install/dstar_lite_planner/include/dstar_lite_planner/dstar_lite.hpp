#pragma once
#include <vector>
#include <map>
#include <queue>
#include <cmath>
#include <limits>
#include <tuple>

namespace dstar_lite_planner {

struct Cell {
  int x, y;
  bool operator==(const Cell & o) const { return x == o.x && y == o.y; }
  bool operator<(const Cell & o) const {
    return std::tie(x, y) < std::tie(o.x, o.y);
  }
};

struct Key {
  double k1, k2;
  bool operator<(const Key & o) const {
    return k1 < o.k1 || (k1 == o.k1 && k2 < o.k2);
  }
  bool operator>(const Key & o) const { return o < *this; }
  bool operator<=(const Key & o) const { return !(o < *this); }
  bool operator==(const Key & o) const { return k1 == o.k1 && k2 == o.k2; }
};

struct PQEntry {
  Key key;
  Cell cell;
  bool operator>(const PQEntry & o) const { return key > o.key; }
};

class DStarLite {
public:
  void initialize(int width, int height);
  void setCost(int x, int y, double cost);
  void setStart(Cell s);
  void setGoal(Cell g);
  bool computeShortestPath();
  std::vector<Cell> extractPath();

private:
  Key calculateKey(Cell s);
  double heuristic(Cell a, Cell b);
  void updateVertex(Cell u);
  std::vector<Cell> getNeighbors(Cell s);
  double edgeCost(Cell a, Cell b);

  int width_, height_;
  std::map<Cell, double> g_, rhs_, cost_;
  std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> U_;
  std::map<Cell, Key> inQueue_;
  Cell start_, goal_;
  double km_ = 0.0;
  const double INF = std::numeric_limits<double>::infinity();
};

}  // namespace dstar_lite_planner