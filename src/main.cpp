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
    string to;
    double weight;
};

// Structure to represent a node in priority queue
struct Node {
    string id;
    double cost;
    bool operator>(const Node &other) const {
        return cost > other.cost;
    }
};

class Graph {
public:
    unordered_map<string, vector<Edge>> adj; // Adjacency list
    unordered_map<string, int> nodeIndex;   // [NEW FEATURE] Mapping names to indices
    vector<string> nodeNames;               // [NEW FEATURE] Stores names of nodes

    // Function to add an edge to the graph
    void addEdge(const string &u, const string &v, double w, bool directed = false) {
        adj[u].push_back({v, w});
        if (!directed) {
            adj[v].push_back({u, w});
        }
        if (nodeIndex.find(u) == nodeIndex.end()) {
            nodeIndex[u] = nodeNames.size();
            nodeNames.push_back(u);
        }
        if (nodeIndex.find(v) == nodeIndex.end()) {
            nodeIndex[v] = nodeNames.size();
            nodeNames.push_back(v);
        }
    }

    // Print adjacency list
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

    // Dijkstra's algorithm with path reconstruction
    vector<string> dijkstra(const string &start, const string &goal) {
        unordered_map<string, string> parent; // Path reconstruction
        unordered_map<string, double> dist;
        set<pair<double, string>> pq;

        for (auto &[node, _] : adj) {
            dist[node] = numeric_limits<double>::infinity();
        }
        dist[start] = 0;
        pq.insert({0, start});

        while (!pq.empty()) {
            string current = pq.begin()->second;
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

        vector<string> path;
        for (string at = goal; at != start; at = parent[at]) {
            if (parent.find(at) == parent.end()) return {};
            path.push_back(at);
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }

    // Floyd-Warshall algorithm
    void floydWarshall() {
        int n = nodeNames.size();
        vector<vector<double>> dist(n, vector<double>(n, numeric_limits<double>::infinity()));

        for (int i = 0; i < n; i++) dist[i][i] = 0;

        for (auto &[u, edges] : adj) {
            int uIdx = nodeIndex[u];
            for (auto &edge : edges) {
                int vIdx = nodeIndex[edge.to];
                dist[uIdx][vIdx] = edge.weight;
            }
        }

        // Algorithm Execution
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

        // [NEW FEATURE] Detect Negative Cycle
        for (int i = 0; i < n; i++) {
            if (dist[i][i] < 0) {
                cout << "Negative cycle detected!\n";
                return;
            }
        }

        // Print result
        cout << "Shortest distances:\n";
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                cout << (dist[i][j] == numeric_limits<double>::infinity() ? "INF" : to_string(dist[i][j])) << " ";
            }
            cout << endl;
        }
    }
};

int main() {
    Graph graph;
    ifstream inputFile("graph_input.txt");

    if (!inputFile) {
        cout << "Error opening file!" << endl;
        return 1;
    }

    bool directed;
    inputFile >> directed;

    string u, v;
    double w;
    while (inputFile >> u >> v >> w) {
        graph.addEdge(u, v, w, directed);
    }
    inputFile.close();

    // Print the graph
    graph.printGraph();

    // User input for Dijkstra
    string start, goal;
    cout << "Enter start and goal nodes: ";
    cin >> start >> goal;

    // Run Dijkstra
    vector<string> pathDijkstra = graph.dijkstra(start, goal);

    // Run Floyd-Warshall
    graph.floydWarshall();

    // [NEW FEATURE] Print Path Function
    auto printPath = [](const string &name, const vector<string> &path) {
        cout << name << " Path: ";
        if (path.empty()) cout << "No path found!";
        for (const string &node : path) cout << node << " ";
        cout << endl;
    };

    printPath("Dijkstra", pathDijkstra);

    return 0;
}