#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <map>
#include <limits>
#include <algorithm>
#include <fstream>

using namespace std;

// Structure to represent an edge in the graph
struct Edge {
    int to, weight;
};

// Structure to represent a node in priority queue
struct Node {
    int id;
    double cost;
    bool operator>(const Node &other) const {
        return cost > other.cost;
    }
};

class Graph {
public:
    unordered_map<int, vector<Edge>> adj; // Adjacency list representation of the graph

    // Function to add an edge to the graph
    void addEdge(int u, int v, int w, bool directed = false) {
        adj[u].push_back({v, w});
        if (!directed) {
            adj[v].push_back({u, w});
        }
    }

    // Function to print the adjacency list of the graph
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

    // Implementation of Dijkstra's algorithm using set for optimization
    vector<int> dijkstra(int start, int goal) {
        unordered_map<int, int> parent; // Stores parent nodes for path reconstruction
        unordered_map<int, double> dist; // Stores shortest distances
        set<pair<double, int>> pq; // Min-heap priority queue using set

        // Initialize distances to infinity
        for (auto &[node, _] : adj) {
            dist[node] = numeric_limits<double>::infinity();
        }
        dist[start] = 0;
        pq.insert({0, start});

        while (!pq.empty()) {
            int current = pq.begin()->second;
            pq.erase(pq.begin()); // Remove the node with the smallest distance

            if (current == goal) break;

            for (const auto &edge : adj[current]) {
                double newDist = dist[current] + edge.weight;
                if (newDist < dist[edge.to]) {
                    pq.erase({dist[edge.to], edge.to}); // Remove outdated entry
                    dist[edge.to] = newDist;
                    parent[edge.to] = current;
                    pq.insert({newDist, edge.to}); // Insert updated entry
                }
            }
        }
        
        // Reconstruct the shortest path from goal to start
        vector<int> path;
        for (int at = goal; at != start; at = parent[at]) {
            if (parent.find(at) == parent.end()) return {}; // No path found
            path.push_back(at);
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }

    // Implementation of Floyd-Warshall algorithm for all-pairs shortest paths
    void floydWarshall(int n) {
        vector<vector<double>> dist(n, vector<double>(n, numeric_limits<double>::infinity()));
        for (int i = 0; i < n; i++) dist[i][i] = 0; // Distance to self is zero
        
        // Initialize distances from adjacency list
        for (auto &[u, edges] : adj) {
            for (auto &edge : edges) {
                dist[u][edge.to] = edge.weight;
            }
        }
        
        // Run Floyd-Warshall algorithm
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
        
        // Print the shortest distances between every pair of vertices
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
    
    // Read input from file instead of manual input
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
    
    // Function to print paths in a structuredrm manner
    auto printPath = [](const string &name, const vector<int> &path) {
        cout << name << " Path: ";
        if (path.empty()) cout << "No path found!";
        for (int node : path) cout << node << " ";
        cout << endl;
    };
    
    printPath("Dijkstra", pathDijkstra);
    return 0;
}