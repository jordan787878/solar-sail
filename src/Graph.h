// Graph.h
#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>

class Graph {
private:
    std::unordered_map<int, int> parent; // Node -> Parent map

public:
    // Constructor
    Graph();

    // Member function to connect two nodes with an edge
    void connect(int node1, int node2);

    // Member function to find the parent node of a given node
    int findParent(int node) const;

    void cleanGraph();
};
