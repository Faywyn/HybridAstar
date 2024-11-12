#ifndef REEDS_SHEPP_H
#define REEDS_SHEPP_H

#include "Node.h"
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <vector>

class ReedsShepp {
public:
  ReedsShepp();
  std::vector<Node> computePath(const Node &start, const Node &goal,
                                int numPoints);
  double distance(const Node &start, const Node &goal);

private:
  ompl::base::ReedsSheppStateSpace reedsSheppSpace;
};

#endif // REEDS_SHEPP_H
