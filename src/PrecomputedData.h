#include "Map.h"
#include "Node.h"
#include "ReedsShepp.h"
#include "Utils.h"
#include <cmath>
#include <unordered_map>
#include <vector>

struct NodeKey {
  int xIndex;
  int yIndex;
  int thetaIndex;
  bool operator==(const NodeKey &other) const {
    return xIndex == other.xIndex && yIndex == other.yIndex &&
           thetaIndex == other.thetaIndex;
  }
};

struct NodeKeyHash {
  std::size_t operator()(const NodeKey &k) const {
    return ((std::hash<int>()(k.xIndex) ^ (std::hash<int>()(k.yIndex) << 1)) >>
            1) ^
           (std::hash<int>()(k.thetaIndex) << 1);
  }
};

class PrecomputedData {
public:
  PrecomputedData(const Node &goal, ReedsShepp &reedsShepp);
  double getHeuristic(const Node &node);

private:
  Node goal;
  ReedsShepp &reedsShepp;

  std::unordered_map<NodeKey, double, NodeKeyHash> heuristicMap;
  std::vector<std::pair<Node, double>> dirsVect;

  void precomputeHeuristics();
  NodeKey getNodeKey(const Node &node);
};
