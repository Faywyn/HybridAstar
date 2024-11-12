#ifndef NODE_H
#define NODE_H

#include <functional>

class Node {
public:
  double x;
  double y;
  double theta;
  bool forward;

  double fScore;

  Node(double x = 0, double y = 0, double theta = 0, bool forward = true);

  bool operator==(const Node &other) const;
  bool operator!=(const Node &other) const;
  bool operator<(const Node &other) const;
  bool operator>(const Node &other) const;
};

namespace std {
template <> struct hash<Node> {
  std::size_t operator()(const Node &node) const {
    return ((std::hash<double>()(node.x) ^
             (std::hash<double>()(node.y) << 1)) >>
            1) ^
           (std::hash<double>()(node.theta) << 1);
  }
};
} // namespace std

#endif // NODE_H
