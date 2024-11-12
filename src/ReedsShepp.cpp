#include "ReedsShepp.h"
#include "Config.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <ompl/base/State.h>

namespace ob = ompl::base;

ReedsShepp::ReedsShepp() : reedsSheppSpace(TURNING_RADIUS) {}

std::vector<Node> ReedsShepp::computePath(const Node &start, const Node &goal,
                                          int numPoints) {
  std::vector<Node> path;

  ob::State *rsStart = reedsSheppSpace.allocState();
  ob::State *rsGoal = reedsSheppSpace.allocState();

  rsStart->as<ob::SE2StateSpace::StateType>()->setXY(start.x, start.y);
  rsStart->as<ob::SE2StateSpace::StateType>()->setYaw(start.theta * M_PI /
                                                      180.0);

  rsGoal->as<ob::SE2StateSpace::StateType>()->setXY(goal.x, goal.y);
  rsGoal->as<ob::SE2StateSpace::StateType>()->setYaw(goal.theta * M_PI / 180.0);

  ob::ReedsSheppStateSpace::ReedsSheppPath rsPath =
      reedsSheppSpace.reedsShepp(rsStart, rsGoal);

  double totalLength = rsPath.length();
  if (totalLength == std::numeric_limits<double>::infinity()) {
    reedsSheppSpace.freeState(rsStart);
    reedsSheppSpace.freeState(rsGoal);
    return path;
  }

  double stepSize = totalLength / numPoints;

  for (int i = 0; i <= numPoints; ++i) {
    double s = i * stepSize;
    if (s > totalLength) {
      s = totalLength;
    }

    ob::State *state = reedsSheppSpace.allocState();
    reedsSheppSpace.interpolate(rsStart, rsGoal, s / totalLength, state);

    double x = state->as<ob::SE2StateSpace::StateType>()->getX();
    double y = state->as<ob::SE2StateSpace::StateType>()->getY();
    double theta =
        state->as<ob::SE2StateSpace::StateType>()->getYaw() * 180.0 / M_PI;

    path.emplace_back(x, y, theta);

    reedsSheppSpace.freeState(state);
  }

  reedsSheppSpace.freeState(rsStart);
  reedsSheppSpace.freeState(rsGoal);

  return path;
}

double ReedsShepp::distance(const Node &start, const Node &goal) {
  ob::State *rsStart = reedsSheppSpace.allocState();
  ob::State *rsGoal = reedsSheppSpace.allocState();

  rsStart->as<ob::SE2StateSpace::StateType>()->setXY(start.x, start.y);
  rsStart->as<ob::SE2StateSpace::StateType>()->setYaw(start.theta * M_PI /
                                                      180.0);

  rsGoal->as<ob::SE2StateSpace::StateType>()->setXY(goal.x, goal.y);
  rsGoal->as<ob::SE2StateSpace::StateType>()->setYaw(goal.theta * M_PI / 180.0);

  double totalLength = reedsSheppSpace.reedsShepp(rsStart, rsGoal).length();

  reedsSheppSpace.freeState(rsStart);
  reedsSheppSpace.freeState(rsGoal);

  return totalLength;
}
