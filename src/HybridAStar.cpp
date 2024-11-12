#include "HybridAStar.h"
#include "Config.h"
#include "Utils.h"
#include <cmath>
#include <iostream>

int hashNode(const Node &node) {
  int x = round(node.x / CELL_SIZE);
  int y = round(node.y / CELL_SIZE);
  int theta = round((180 + normalizeAngle(node.theta)) / THETA_SIZE);
  int speed = node.forward ? 1 : 0;

  int maxY = std::ceil(WINDOW_HEIGHT / CELL_SIZE);
  int maxTheta = std::ceil(360 / THETA_SIZE);

  return speed + 2 * (theta + maxTheta * (y + maxY * x));
}

HybridAStar::HybridAStar()
    : map(nullptr), pathFound(false), reedsShepp(), precomputedData(nullptr) {}

void HybridAStar::initialize(Node start, Node goal, Map &map) {
  this->map = &map;
  this->goalNode = goal;

  if (precomputedData != nullptr) {
    delete precomputedData;
  }
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  std::cout << "Initializing precomputed data..." << std::endl;
  precomputedData = new PrecomputedData(goal, reedsShepp);
  std::cout << "Precomputed data initialized ("
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - begin)
                   .count()
            << "ms)" << std::endl;

  openSet = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>();
  cameFrom.clear();
  gScore.clear();
  visitedNodes.clear();

  start.fScore = heuristic(start);
  openSet.push(start);
  cameFrom[start] = start;
  gScore[hashNode(start)] = 0;
  pathFound = false;
}

bool HybridAStar::step() {
  if (openSet.empty() || pathFound) {
    return false;
  }

  current = openSet.top();
  openSet.pop();

  visitedNodes.push_back(current);

  if (goalReached(current)) {
    pathFound = true;
    return false;
  }

  std::vector<Node> successors;
  for (int dir = 0; dir < 2; dir++) {
    bool forward = dir == 0;
    double newSpeed = forward ? 1 : -1;

    for (double steeringAngle = -STEER_ANGLE; steeringAngle <= STEER_ANGLE;
         steeringAngle += STEER_ANGLE) {

      double thetaRad = current.theta * M_PI / 180.0;

      double newX = current.x + cos(thetaRad) * STEP_SIZE * newSpeed;
      double newY = current.y + sin(thetaRad) * STEP_SIZE * newSpeed;
      double newTheta = current.theta + steeringAngle * newSpeed;
      newTheta = normalizeAngle(newTheta);

      Node neighbor(newX, newY, newTheta, forward);

      std::vector<Node> path = reedsShepp.computePath(current, neighbor, 3);

      if (!path.empty() && isCollisionFreePath(path)) {
        successors.push_back(neighbor);
      }
    }

    for (auto &neighbor : successors) {
      size_t hash = hashNode(neighbor);
      double tentativeGScore =
          gScore[hashNode(current)] + cost(current, neighbor);

      if (gScore.find(hash) == gScore.end() || tentativeGScore < gScore[hash]) {
        cameFrom[neighbor] = current;
        gScore[hash] = tentativeGScore;
        neighbor.fScore = gScore[hash] + heuristic(neighbor);
        openSet.push(neighbor);
      }
    }
  }

  return true;
}

bool HybridAStar::goalReached(const Node &node) {
  if (normalizeAngle(node.theta) != normalizeAngle(goalNode.theta)) {
    return false;
  }
  return heuristic(node) <= 1;
}

bool HybridAStar::isCollisionFreePath(const std::vector<Node> &path) {
  for (const auto &node : path) {
    if (map->isCollision(node)) {
      return false;
    }
  }
  return true;
}

std::vector<Node> HybridAStar::reconstructPath() {
  std::vector<Node> path;
  if (!pathFound)
    return path;

  Node node = current;
  while (!(node == cameFrom[node])) {
    path.push_back(node);
    node = cameFrom[node];
  }
  path.push_back(node);
  std::reverse(path.begin(), path.end());
  return path;
}

bool HybridAStar::isPathFound() const { return pathFound; }

std::vector<Node> HybridAStar::getVisitedNodes() const { return visitedNodes; }

double HybridAStar::cost(const Node &a, const Node &b) {
  double dist = reedsShepp.distance(a, b);
  double total = dist;

  if (a.forward != b.forward) {
    total *= 2;
  }

  return total;
}

double HybridAStar::heuristic(const Node &a) {
  return precomputedData->getHeuristic(a);
}
