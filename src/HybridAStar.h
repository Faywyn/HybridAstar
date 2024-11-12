#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include "Config.h"
#include "Map.h"
#include "Node.h"
#include "PrecomputedData.h"
#include "ReedsShepp.h"
#include <queue>
#include <unordered_map>
#include <vector>

class HybridAStar {
public:
  HybridAStar();
  void initialize(Node start, Node goal, Map &map);
  bool step();
  std::vector<Node> reconstructPath();
  bool isPathFound() const;
  std::vector<Node> getVisitedNodes() const;

private:
  double heuristic(const Node &a);
  double cost(const Node &a, const Node &b);
  bool isCollisionFreePath(const std::vector<Node> &path);
  bool goalReached(const Node &node);

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
  std::unordered_map<Node, Node> cameFrom;
  std::unordered_map<int, double> gScore;
  Node goalNode;
  Map *map;
  bool pathFound;
  Node current;
  std::vector<Node> visitedNodes;

  ReedsShepp reedsShepp;
  PrecomputedData *precomputedData;
};
#endif // HYBRID_A_STAR_H
