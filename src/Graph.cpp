// Graph.cpp
#include "Graph.h"

// Constructor
Graph::Graph() {}

// Member function to connect two nodes with an edge
void Graph::connect(int node1, int node2) {
    // Assuming node1 and node2 are integers
    parent[node1] = node2;
}

// Member function to find the parent node of a given node
int Graph::findParent(int node) const {
    auto it = parent.find(node);
    if (it != parent.end()) {
        return it->second;
    } else {
        // std::cerr << "Node " << node << " not found in the graph." << std::endl;
        return -1; // Assuming -1 represents an invalid node value
    }
}

void Graph::cleanGraph(){
    parent.clear();
}
