#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <cmath>
#include <limits>
#include <algorithm>
#include <fstream>

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
public:
    unordered_map<int, vector<Edge>> adj;

    void addEdge(int u, int v, int w, bool directed = false) {
        adj[u].push_back({v, w});
        if (!directed) {
            adj[v].push_back({u, w});
        }
    }

    void printGraph() {
        cout << "Graph adjacency list:\n";
        for (auto &[node, edges] : adj) {
            cout << "Node " << node << ": ";
            for (auto &edge : edges) {
                cout << "(" << edge.to << ", " << edge.weight << ") ";
            }
            cout << endl;
        }
    }

    vector<int> dijkstra(int start, int goal) {
        unordered_map<int, int> parent;
        unordered_map<int, double> dist;
        set<pair<double, int>> pq;

        for (auto &[node, _] : adj) {
            dist[node] = numeric_limits<double>::infinity();
        }
        dist[start] = 0;
        pq.insert({0, start});

        while (!pq.empty()) {
            int current = pq.begin()->second;
            pq.erase(pq.begin());

            if (current == goal) break;

            for (const auto &edge : adj[current]) {
                double newDist = dist[current] + edge.weight;
                if (newDist < dist[edge.to]) {
                    pq.erase({dist[edge.to], edge.to});
                    dist[edge.to] = newDist;
                    parent[edge.to] = current;
                    pq.insert({newDist, edge.to});
                }
            }
        }
        
        vector<int> path;
        for (int at = goal; at != start; at = parent[at]) {
            if (parent.find(at) == parent.end()) return {};
            path.push_back(at);
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }

    void floydWarshall(int n) {
        vector<vector<double>> dist(n, vector<double>(n, numeric_limits<double>::infinity()));
        for (int i = 0; i < n; i++) dist[i][i] = 0;
        
        for (auto &[u, edges] : adj) {
            for (auto &edge : edges) {
                dist[u][edge.to] = edge.weight;
            }
        }
        
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (dist[i][k] != numeric_limits<double>::infinity() &&
                        dist[k][j] != numeric_limits<double>::infinity() &&
                        dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }
        
        cout << "Shortest distances between every pair of vertices:\n";
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (dist[i][j] == numeric_limits<double>::infinity())
                    cout << "INF ";
                else
                    cout << dist[i][j] << " ";
            }
            cout << endl;
        }
    }
};

int main() {
    Graph graph;
    int edges, u, v, w, nodes;
    bool directed;
    
    ifstream inputFile("graph_input.txt");
    if (!inputFile) {
        cout << "Error opening file!" << endl;
        return 1;
    }
    
    inputFile >> nodes >> edges >> directed;
    
    for (int i = 0; i < edges; i++) {
        inputFile >> u >> v >> w;
        graph.addEdge(u, v, w, directed);
    }
    inputFile.close();

    int start, goal;
    cout << "Enter start and goal nodes: ";
    cin >> start >> goal;

    vector<int> pathDijkstra = graph.dijkstra(start, goal);
    graph.floydWarshall(nodes);
    
    auto printPath = [](const string &name, const vector<int> &path) {
        cout << name << " Path: ";
        if (path.empty()) cout << "No path found!";
        for (int node : path) cout << node << " ";
        cout << endl;
    };
    
    printPath("Dijkstra", pathDijkstra);
    return 0;
}