#include "Map.h"
#include "Config.h"
#include <iostream>

Map::Map() {
  obstacles.clear();

  sf::RectangleShape obstacle1(sf::Vector2f(200, 50));
  obstacle1.setPosition(300, 200);
  obstacle1.setFillColor(sf::Color::Black);
  obstacles.push_back(obstacle1);

  sf::RectangleShape obstacle2(sf::Vector2f(50, 500));
  obstacle2.setPosition(500, 100);
  obstacle2.setFillColor(sf::Color::Black);
  obstacles.push_back(obstacle2);

  sf::RectangleShape obstacle3(sf::Vector2f(300, 50));
  obstacle3.setPosition(300, 600);
  obstacle3.setFillColor(sf::Color::Black);
  obstacles.push_back(obstacle3);

  sf::RectangleShape obstacle4(sf::Vector2f(300, 50));
  obstacle4.setPosition(500, 800);
  obstacle4.setFillColor(sf::Color::Black);
  obstacles.push_back(obstacle4);

  sf::RectangleShape obstacle5(sf::Vector2f(50, 400));
  obstacle5.setPosition(800, 300);
  obstacle5.setFillColor(sf::Color::Black);
  obstacles.push_back(obstacle5);
}

bool Map::isCollision(const Node &node) const {
  if (node.x < 0 || node.x >= WINDOW_WIDTH || node.y < 0 ||
      node.y >= WINDOW_HEIGHT) {
    return true;
  }

  for (const auto &obstacle : obstacles) {
    double obstacleX = obstacle.getPosition().x;
    double obstacleY = obstacle.getPosition().y;
    double obstacleWidth = obstacle.getSize().x;
    double obstacleHeight = obstacle.getSize().y;

    if (node.x >= obstacleX && node.x <= obstacleX + obstacleWidth &&
        node.y >= obstacleY && node.y <= obstacleY + obstacleHeight) {
      return true;
    }
  }
  return false;
}

void Map::draw(sf::RenderWindow &window) {
  for (const auto &obstacle : obstacles) {
    window.draw(obstacle);
  }
}
