// Utils.h
#ifndef UTILS_H
#define UTILS_H

#include "Node.h"
#include <SFML/Graphics.hpp>
#include <cmath>

inline double normalizeAngle(double angle) {
  while (angle <= -180.0)
    angle += 360.0;
  while (angle > 180.0)
    angle -= 360.0;
  return angle;
}

inline void drawArrow(sf::RenderWindow &window, sf::Vector2f position,
                      float rotation, float length, float thickness,
                      sf::Color color = sf::Color::Red, bool outline = false) {
  sf::ConvexShape arrow;
  arrow.setPointCount(7);
  arrow.setPoint(0, sf::Vector2f(0, 0));
  arrow.setPoint(1, sf::Vector2f(-length / 2, thickness));
  arrow.setPoint(2, sf::Vector2f(-length / 2, thickness / 2));
  arrow.setPoint(3, sf::Vector2f(-length, thickness / 2));
  arrow.setPoint(4, sf::Vector2f(-length, -thickness / 2));
  arrow.setPoint(5, sf::Vector2f(-length / 2, -thickness / 2));
  arrow.setPoint(6, sf::Vector2f(-length / 2, -thickness));

  arrow.setFillColor(color);
  arrow.setOrigin(-length / 2, 0);
  arrow.setPosition(position);
  arrow.setRotation(rotation);

  if (outline) {
    arrow.setOutlineThickness(1);
    arrow.setOutlineColor(sf::Color::Black);
  }

  window.draw(arrow);
}

inline void drawThickPath(sf::RenderWindow &window,
                          const std::vector<Node> &rsPath, float thickness) {
  for (size_t i = 0; i < rsPath.size() - 1; ++i) {
    sf::Vector2f point1(rsPath[i].x, rsPath[i].y);
    sf::Vector2f point2(rsPath[i + 1].x, rsPath[i + 1].y);

    sf::Vector2f direction = point2 - point1;
    float length =
        std::sqrt(direction.x * direction.x + direction.y * direction.y);
    sf::Vector2f unitDirection = direction / length;

    sf::Vector2f offset(-unitDirection.y * thickness / 2,
                        unitDirection.x * thickness / 2);

    sf::ConvexShape thickSegment;
    thickSegment.setPointCount(4);
    thickSegment.setPoint(0, point1 + offset);
    thickSegment.setPoint(1, point2 + offset);
    thickSegment.setPoint(2, point2 - offset);
    thickSegment.setPoint(3, point1 - offset);

    thickSegment.setFillColor(sf::Color::Red);
    window.draw(thickSegment);
  }
}

#endif // UTILS_H
