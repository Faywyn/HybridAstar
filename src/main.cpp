#include "HybridAStar.h"
#include "Map.h"
#include "Utils.h"
#include <SFML/Graphics.hpp>
#include <iostream>

typedef struct config {
  Node start;
  Node goal;
} config;

config randomPos(Map *map) {
  Node start;
  Node goal;
  int resX = WINDOW_WIDTH / CELL_SIZE - 1;
  int resY = WINDOW_HEIGHT / CELL_SIZE - 1;
  int resTheta = 360 / THETA_SIZE;
  do {
    start = Node(rand() % resX * CELL_SIZE, rand() % resY * CELL_SIZE,
                 rand() % resTheta * THETA_SIZE, true);
    goal = Node(rand() % resX * CELL_SIZE, rand() % resY * CELL_SIZE,
                rand() % resTheta * THETA_SIZE, true);

  } while (map->isCollision(start) || map->isCollision(goal));
  start.theta = normalizeAngle(start.theta);
  goal.theta = normalizeAngle(goal.theta);

  std::cout << "Start pos: " << start.x << ", " << start.y << ", "
            << start.theta << std::endl;
  std::cout << "Goal pos: " << goal.x << ", " << goal.y << ", " << goal.theta
            << std::endl;

  return {start, goal};
}

int main() {
  srand(time(nullptr));

  sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT),
                          "Hybrid A* - Reeds-Shepp");

  Map map;

  config pos = randomPos(&map);
  Node start = pos.start;
  Node goal = pos.goal;

  HybridAStar planner;
  planner.initialize(start, goal, map);

  sf::Clock clock;
  float updateTime = 0.1f;

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();

      if (event.type == sf::Event::KeyPressed &&
          event.key.code == sf::Keyboard::Space) {
        std::cout << "Starting over ..." << std::endl;
        config pos = randomPos(&map);
        start = pos.start;
        goal = pos.goal;

        planner.initialize(start, goal, map);
        clock.restart();
      }
    }

    if (!planner.isPathFound()) {
      bool running = planner.step();

      if (clock.getElapsedTime().asSeconds() >= updateTime || !running) {
        clock.restart();

        window.clear(sf::Color::White);

        map.draw(window);

        drawArrow(window, sf::Vector2f(start.x, start.y), start.theta, 20, 5,
                  sf::Color::Green, true);
        drawArrow(window, sf::Vector2f(goal.x, goal.y), goal.theta, 20, 5,
                  sf::Color::Red, true);

        std::vector<Node> visitedNodes = planner.getVisitedNodes();

        sf::VertexArray visitedPoints(sf::Points, visitedNodes.size());
        for (size_t i = 0; i < visitedNodes.size(); ++i) {
          visitedPoints[i].position =
              sf::Vector2f(visitedNodes[i].x, visitedNodes[i].y);
          visitedPoints[i].color = sf::Color(0, 0, 255, 30);
        }
        window.draw(visitedPoints);

        if (planner.isPathFound()) {
          std::vector<Node> path = planner.reconstructPath();

          sf::VertexArray pathLines(sf::LineStrip);
          for (const auto &node : path) {
            pathLines.append(
                sf::Vertex(sf::Vector2f(node.x, node.y), sf::Color::Red));
          }
          window.draw(pathLines);
        }

        window.display();
      }
    } else {
      if (clock.getElapsedTime().asSeconds() >= updateTime) {
        clock.restart();

        window.clear(sf::Color::White);

        map.draw(window);

        drawArrow(window, sf::Vector2f(start.x, start.y), start.theta, 20, 5,
                  sf::Color::Green, true);
        drawArrow(window, sf::Vector2f(goal.x, goal.y), goal.theta, 20, 5,
                  sf::Color::Red, true);

        std::vector<Node> visitedNodes = planner.getVisitedNodes();

        sf::VertexArray visitedPoints(sf::Points, visitedNodes.size());
        for (size_t i = 0; i < visitedNodes.size(); ++i) {
          visitedPoints[i].position =
              sf::Vector2f(visitedNodes[i].x, visitedNodes[i].y);
          visitedPoints[i].color = sf::Color(0, 0, 255, 50);
        }
        window.draw(visitedPoints);

        std::vector<Node> path = planner.reconstructPath();

        ReedsShepp reedsShepp;
        for (size_t i = 0; i < path.size() - 1; ++i) {
          std::vector<Node> rsPath =
              reedsShepp.computePath(path[i], path[i + 1], 10);

          drawThickPath(window, rsPath, 2);
        }

        window.display();
      }
    }
  }

  return 0;
}
