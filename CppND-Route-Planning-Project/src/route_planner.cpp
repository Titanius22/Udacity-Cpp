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
    return ((i->h_value + i->g_value) > (j->h_value + j->g_value));
}


RouteModel::Node *RoutePlanner::NextNode() {
    sort(this->open_list.begin(), this->open_list.end(), NextNodeSorter);
    
    RouteModel::Node * nextNode = this->open_list.back();
    
    ////////////////std::cout << "nextNode's g_value=" << nextNode->g_value << std::endl;
    ////////////////std::cout << "nextNode's h_value=" << nextNode->h_value << std::endl;
    ////////////////std::cout << "nextNode's g+h_value=" << (nextNode->h_value + nextNode->g_value) << std::endl;
  
    
    this->open_list.pop_back();
    ////////////////if(open_list.size() > 0){
    ////////////////   std::cout << "sorted_end-1=" << (this->open_list.back()->h_value + this->open_list.back()->g_value) << std::endl;
    ////////////////}
      
    //open_list.clear(); //////////////////////////////////////////////////////////////////////////////////////////////////
  
    //nextNode->visited = true; ///////////////////////////////////////////////////////////////////////////////////////////
    return nextNode;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    RouteModel::Node *buff_node = current_node;
    
    //std::cout << "erik test 1" << std::endl;
    while(buff_node != start_node){
        path_found.push_back(*buff_node);
        buff_node = buff_node->parent;
    }
    path_found.push_back(*start_node);
    ////////////////std::cout << "erik test 2" << std::endl;
    std::reverse(path_found.begin(), path_found.end());
    
    distance = current_node->g_value; // the g_value is already the distance from start

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_listand return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    ////////////////RouteModel::Node *last_node = nullptr;
    
    current_node = start_node;
    AddNeighbors(current_node);
    
    while(open_list.size() > 0){
        ////////////////std::cout << open_list.size() << std::endl;
        
        ////////////////last_node = current_node;
        current_node = NextNode();
        
        ////////////////bool isNeighbors = false;
        ////////////////for(RouteModel::Node *neighbor_node : last_node->neighbors){
        ////////////////    if(current_node==neighbor_node){
        ////////////////        isNeighbors = true;
        ////////////////    }
        ////////////////}
        ////////////////if(!isNeighbors){
        ////////////////    std::cout << "current_node and last_node aren't neighbors!!!!!!!!!!!!!!!!!!" << std::endl;
        ////////////////}
        
        ////////////////std::cout << "parent_x=" << current_node->parent->x << " parent_y=" << current_node->parent->y << std::endl;
        ////////////////std::cout << "x=" << current_node->x << " y=" << current_node->y << std::endl;
      
        if(current_node == end_node){
            ////////////////std::cout << "end found" << std::endl;
            m_Model.path = ConstructFinalPath(current_node);
            ////////////////std::cout << "end found" << std::endl;
            break;
        }
      
        AddNeighbors(current_node);
    }
}