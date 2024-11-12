#ifndef MAP_H
#define MAP_H

#include "Node.h"
#include <SFML/Graphics.hpp>
#include <vector>

class Map {
public:
  Map();
  bool isCollision(const Node &node) const;
  void draw(sf::RenderWindow &window);

private:
  std::vector<sf::RectangleShape> obstacles;
};

#endif // MAP_H
