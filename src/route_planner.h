#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    float GetDistance() const {return distance;}
    float GetDijkstraDistance() const {return dijkstra_distance;}

    void AStarSearch();
    void DijkstraSearch();

    // The following methods have been made public so we can test them individually.
    void AddNeighbors(RouteModel::Node *current_node);
    void AddNeighborsDijkstra(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();
    RouteModel::Node *NextNodeDijkstra();

  private:
    std::vector<RouteModel::Node*> open_list;
    std::vector<RouteModel::Node*> dijkstra_open_list;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    float distance = 0.0f;
    float dijkstra_distance = 0.0f;
    RouteModel &m_Model;
};

#endif