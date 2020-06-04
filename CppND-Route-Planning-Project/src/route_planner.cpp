#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return end_node->distance(* node);
}




void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();    
    for(auto node : current_node->neighbors){
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        open_list.emplace_back(node);
        node->visited = true;
    }
}



RouteModel::Node *RoutePlanner::NextNode() {
	std::sort(open_list.begin(), open_list.end(), [](const auto& a, const auto& b) {
        return a->h_value + a->g_value > b->h_value + b->g_value;
        });
    
    auto *node = open_list.back();
    open_list.pop_back();
    return node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
  	while (current_node != start_node) {
        path_found.insert(path_found.begin(), *current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.insert(path_found.begin(), *start_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
  current_node = start_node;
    current_node->visited = true;
    open_list.emplace_back(start_node);
    while (open_list.size() != 0) {
        if (current_node->x == end_node->x && current_node->y == end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
        current_node = NextNode();
 }


}