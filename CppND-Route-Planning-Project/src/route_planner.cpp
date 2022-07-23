#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float hValue;
  
    hValue = this->end_node->distance(*node);
  
    return hValue;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
  
    for(RouteModel::Node * neighbor_node : current_node->neighbors){
        if(!(neighbor_node->visited)){
            neighbor_node->parent = current_node;
            neighbor_node->h_value = this->CalculateHValue(neighbor_node);

            // this is path distance
            neighbor_node->g_value = current_node->g_value + current_node->distance(*neighbor_node);
            
            neighbor_node->visited = true;
            this->open_list.push_back(neighbor_node);
        }
    }
}


// function for sorting the open_list 
bool NextNodeSorter(const RouteModel::Node* i, const RouteModel::Node* j){
    float iCombined = i->h_value + i->g_value;
    float jCombined = j->h_value + j->g_value;
    
    if(iCombined == jCombined){
        return (i->h_value > j->h_value);
    }
    else{
        return (iCombined > jCombined);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    sort(this->open_list.begin(), this->open_list.end(), NextNodeSorter);
    
    RouteModel::Node * nextNode = this->open_list.back();
    
    this->open_list.pop_back();
    return nextNode;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    RouteModel::Node *buff_node = current_node;
    
    while(buff_node != start_node){

        path_found.push_back(*buff_node);
        buff_node = buff_node->parent;
    }
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());
    
    distance = current_node->g_value; // the g_value is already the distance from start

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    current_node = start_node;
    start_node->visited=true; //must be marked so its not swept up with AddNeighbors() in the second node
    AddNeighbors(current_node);
    
    while(open_list.size() > 0){
        current_node = NextNode();
      
        if(current_node == end_node){
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
      
        AddNeighbors(current_node);
    }
}