#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

using namespace std;

struct Edge {
    int to, weight;
};

struct Node {
    int id;
    double cost;
    bool operator>(const Node &other) const {
        return cost > other.cost;
    }
};