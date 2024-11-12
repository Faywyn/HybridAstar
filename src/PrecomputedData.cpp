#include "PrecomputedData.h"
#include "Config.h"

PrecomputedData::PrecomputedData(const Node &goal, ReedsShepp &reedsShepp)
    : goal(goal), reedsShepp(reedsShepp) {
  precomputeHeuristics();
}

NodeKey PrecomputedData::getNodeKey(const Node &node) {
  int xIndex = static_cast<int>(round(node.x / CELL_SIZE));
  int yIndex = static_cast<int>(round(node.y / CELL_SIZE));
  int thetaIndex =
      static_cast<int>(round(normalizeAngle(node.theta) / THETA_SIZE));
  return {xIndex, yIndex, thetaIndex};
}

void PrecomputedData::precomputeHeuristics() {
  heuristicMap.clear();
  int numXCells = static_cast<int>(WINDOW_WIDTH / CELL_SIZE);
  int numYCells = static_cast<int>(WINDOW_HEIGHT / CELL_SIZE);
  int numThetaCells = static_cast<int>(360 / THETA_SIZE);

  for (int xIndex = 0; xIndex <= numXCells; ++xIndex) {
    double x = xIndex * CELL_SIZE;
    for (int yIndex = 0; yIndex <= numYCells; ++yIndex) {
      double y = yIndex * CELL_SIZE;
      for (int thetaIndex = 0; thetaIndex < numThetaCells; ++thetaIndex) {
        double theta = thetaIndex * THETA_SIZE;
        Node node(x, y, theta, true);
        double h = reedsShepp.distance(node, goal);
        NodeKey key = {xIndex, yIndex, thetaIndex};
        heuristicMap[key] = h;
      }
    }
  }
}

double PrecomputedData::getHeuristic(const Node &node) {
  NodeKey key = getNodeKey(node);
  auto it = heuristicMap.find(key);
  if (it != heuristicMap.end()) {
    return it->second;
  } else {
    return reedsShepp.distance(node, goal);
  }
}
