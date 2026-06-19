#include "route_planner.h"
#include <algorithm>
#include <chrono>
#include <iostream>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// ─── SHARED ───────────────────────────────────────────────────────────────────

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*this->end_node);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  path_found.push_back(*current_node);
  while (current_node != this->start_node) {
    distance += current_node->distance(*current_node->parent);
    path_found.push_back(*current_node->parent);
    current_node = current_node->parent;
  }
  std::reverse(path_found.begin(), path_found.end());
  distance *= m_Model.MetricScale();
  return path_found;
}

// ─── A* SEARCH ────────────────────────────────────────────────────────────────

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->g_value =
        current_node->g_value + neighbor->distance(*current_node);
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->visited = true;
    this->open_list.emplace_back(neighbor);
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(this->open_list.begin(), this->open_list.end(),
            [](const RouteModel::Node *v1, const RouteModel::Node *v2) {
              return v1->h_value + v1->g_value > v2->h_value + v2->g_value;
            });
  auto next_node = this->open_list.back();
  this->open_list.pop_back();
  return next_node;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

  this->start_node->visited = true;
  this->open_list.push_back(this->start_node);

  auto start_time = std::chrono::high_resolution_clock::now();

  while (this->open_list.size() > 0) {
    RouteModel::Node *next_node = NextNode();
    if (next_node->x == this->end_node->x &&
        next_node->y == this->end_node->y) {
      m_Model.path = ConstructFinalPath(next_node);
      break;
    } else {
      AddNeighbors(next_node);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  std::cout << "\n========== A* SEARCH ==========\n";
  std::cout << "Distance : " << distance << " meters\n";
  std::cout << "Time     : " << duration.count() << " microseconds\n";
  std::cout << "================================\n";
}

// ─── DIJKSTRA SEARCH ──────────────────────────────────────────────────────────
// Dijkstra is A* with h_value = 0 (no heuristic).
// It explores in all directions equally, so it's slower but still finds
// the optimal path.

void RoutePlanner::AddNeighborsDijkstra(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    float new_g = current_node->g_value + neighbor->distance(*current_node);
    // Only update if we found a shorter path to this neighbor
    if (!neighbor->visited || new_g < neighbor->g_value) {
      neighbor->parent = current_node;
      neighbor->g_value = new_g;
      neighbor->h_value = 0.0f; // No heuristic — this is what makes it Dijkstra
      neighbor->visited = true;
      this->dijkstra_open_list.emplace_back(neighbor);
    }
  }
}

RouteModel::Node *RoutePlanner::NextNodeDijkstra() {
  // Sort by g_value only (no h_value)
  std::sort(this->dijkstra_open_list.begin(), this->dijkstra_open_list.end(),
            [](const RouteModel::Node *v1, const RouteModel::Node *v2) {
              return v1->g_value > v2->g_value;
            });
  auto next_node = this->dijkstra_open_list.back();
  this->dijkstra_open_list.pop_back();
  return next_node;
}

void RoutePlanner::DijkstraSearch() {
  // Reset visited flags for all nodes so Dijkstra runs fresh
  for (auto &node : m_Model.SNodes()) {
    node.visited = false;
    node.parent = nullptr;
    node.g_value = 0.0f;
    node.h_value = 0.0f;
  }

  this->start_node->visited = true;
  this->dijkstra_open_list.push_back(this->start_node);

  auto start_time = std::chrono::high_resolution_clock::now();

  while (this->dijkstra_open_list.size() > 0) {
    RouteModel::Node *next_node = NextNodeDijkstra();
    if (next_node->x == this->end_node->x &&
        next_node->y == this->end_node->y) {
      // Save distances before ConstructFinalPath overwrites distance
      auto path = ConstructFinalPath(next_node);
      dijkstra_distance = distance;
      break;
    } else {
      AddNeighborsDijkstra(next_node);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  std::cout << "\n========== DIJKSTRA SEARCH ==========\n";
  std::cout << "Distance : " << dijkstra_distance << " meters\n";
  std::cout << "Time     : " << duration.count() << " microseconds\n";
  std::cout << "=====================================\n";
}