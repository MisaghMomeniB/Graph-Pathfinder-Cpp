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

class Graph {
    unordered_map<int, vector<Edge>> adj;
public:
    void addEdge(int u, int v, int w) {
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
    }

    vector<int> dijkstra(int start, int goal) {
        unordered_map<int, int> parent;
        unordered_map<int, double> dist;
        priority_queue<Node, vector<Node>, greater<Node>> pq;
        
        for (auto &[node, _] : adj) dist[node] = numeric_limits<double>::infinity();
        
        dist[start] = 0;
        pq.push({start, 0});
        
        while (!pq.empty()) {
            int current = pq.top().id;
            double currentCost = pq.top().cost;
            pq.pop();
            
            if (current == goal) break;
            
            for (const auto &edge : adj[current]) {
                double newDist = currentCost + edge.weight;
                if (newDist < dist[edge.to]) {
                    dist[edge.to] = newDist;
                    parent[edge.to] = current;
                    pq.push({edge.to, newDist});
                }
            }
        }