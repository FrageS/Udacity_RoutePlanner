#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return (*node).distance(*end_node); 
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    (*current_node).FindNeighbors();
  
    for(RouteModel::Node *neighbor : (*current_node).neighbors)
    {
        // Set the parrent of each neighbor to current node
        neighbor->parent = current_node;
        // Calculate H - Value (neighbor is node to calculate distance to end node)
        neighbor->h_value = CalculateHValue(neighbor);
        // Calculate g-value
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}

bool Compare(RouteModel::Node *a, RouteModel::Node *b)
{
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1>f2;
}

RouteModel::Node *RoutePlanner::NextNode() {    
    std::sort(open_list.begin(), open_list.end(),Compare);

    auto *lowest = open_list.back();
    open_list.pop_back();
    return lowest;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    while(current_node->parent)
    {
        distance += current_node->distance(*(current_node->parent));
        path_found.push_back((*current_node));
        current_node = current_node->parent;
    }

    path_found.push_back((*current_node));
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    //Adding start node to openlist
    open_list.push_back(start_node);
    start_node->visited = true;

    while(open_list.size() > 0)
    {   
        //Get next node
        current_node = NextNode();
        if(current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // Get all neighbors
        AddNeighbors(current_node); 
    }
    
}