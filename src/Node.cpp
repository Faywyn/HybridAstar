#include "Node.h"

Node::Node(double x, double y, double theta, bool forward)
    : x(x), y(y), theta(theta), forward(forward) {}

bool Node::operator==(const Node &other) const {
  return x == other.x && y == other.y && theta == other.theta &&
         forward == other.forward;
}

bool Node::operator!=(const Node &other) const { return !(*this == other); }

bool Node::operator>(const Node &other) const { return fScore > other.fScore; }
