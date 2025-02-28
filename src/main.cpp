#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm> // Added for std::reverse

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

        vector<int> path;
        for (int at = goal; at != start; at = parent[at]) {
            if (parent.find(at) == parent.end()) return {};
            path.push_back(at);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end()); // Fixed reverse function
        return path;
    }

    vector<int> aStar(int start, int goal, unordered_map<int, pair<int, int>> &coordinates) {
        auto heuristic = [&](int a, int b) {
            return sqrt(pow(coordinates[a].first - coordinates[b].first, 2) +
                        pow(coordinates[a].second - coordinates[b].second, 2));
        };
        
        unordered_map<int, int> parent;
        unordered_map<int, double> gScore, fScore;
        priority_queue<Node, vector<Node>, greater<Node>> pq;
        
        for (auto &[node, _] : adj) {
            gScore[node] = numeric_limits<double>::infinity();
            fScore[node] = numeric_limits<double>::infinity();
        }
        
        gScore[start] = 0;
        fScore[start] = heuristic(start, goal);
        pq.push({start, fScore[start]});
        
        while (!pq.empty()) {
            int current = pq.top().id;
            pq.pop();
            
            if (current == goal) break;
            
            for (const auto &edge : adj[current]) {
                double tentative_gScore = gScore[current] + edge.weight;
                if (tentative_gScore < gScore[edge.to]) {
                    parent[edge.to] = current;
                    gScore[edge.to] = tentative_gScore;
                    fScore[edge.to] = gScore[edge.to] + heuristic(edge.to, goal);
                    pq.push({edge.to, fScore[edge.to]});
                }
            }
        }
        
        vector<int> path;
        for (int at = goal; at != start; at = parent[at]) {
            if (parent.find(at) == parent.end()) return {};
            path.push_back(at);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end()); // Fixed reverse function
        return path;
    }
};

int main() {
    Graph graph;
    graph.addEdge(1, 2, 1);
    graph.addEdge(1, 3, 4);
    graph.addEdge(2, 3, 2);
    graph.addEdge(2, 4, 5);
    graph.addEdge(3, 4, 1);
    
    unordered_map<int, pair<int, int>> coordinates = {
        {1, {0, 0}}, {2, {1, 2}}, {3, {2, 2}}, {4, {3, 0}}
    };
    
    vector<int> pathDijkstra = graph.dijkstra(1, 4);
    vector<int> pathAStar = graph.aStar(1, 4, coordinates);
    
    cout << "Dijkstra Path: ";
    for (int node : pathDijkstra) cout << node << " ";
    cout << endl;
    
    cout << "A* Path: ";
    for (int node : pathAStar) cout << node << " ";
    cout << endl;
    
    return 0;
}