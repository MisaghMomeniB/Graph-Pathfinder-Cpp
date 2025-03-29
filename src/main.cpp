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

struct Edge {
    string to;
    double weight;
};

// Structure to represent an edge in the graph
struct Node {
    string id;
    double cost;
    bool operator>(const Node &other) const {
        return cost > other.cost;
    }
};

class Graph {
public:
    unordered_map<string, vector<Edge>> adj;
    unordered_map<string, int> nodeIndex;
    vector<string> nodeNames;
    vector<vector<int>> next;

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

    vector<string> reconstructPath(const string &start, const string &goal) {
        int u = nodeIndex[start], v = nodeIndex[goal];
        if (next[u][v] == -1) return {};
        vector<string> path = {start};
        while (u != v) {
            u = next[u][v];
            path.push_back(nodeNames[u]);
        }
        return path;
    }

    void floydWarshall() {
        int n = nodeNames.size();
        vector<vector<double>> dist(n, vector<double>(n, numeric_limits<double>::infinity()));
        next.assign(n, vector<int>(n, -1));
        
        for (int i = 0; i < n; i++) {
            dist[i][i] = 0;
            next[i][i] = i;
        }

        for (auto &[u, edges] : adj) {
            int uIdx = nodeIndex[u];
            for (auto &edge : edges) {
                int vIdx = nodeIndex[edge.to];
                dist[uIdx][vIdx] = edge.weight;
                next[uIdx][vIdx] = vIdx;
            }
        }

        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (dist[i][k] != numeric_limits<double>::infinity() &&
                        dist[k][j] != numeric_limits<double>::infinity() &&
                        dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        next[i][j] = next[i][k];
                    }
                }
            }
        }

        for (int i = 0; i < n; i++) {
            if (dist[i][i] < 0) {
                cout << "Negative cycle detected!\n";
                return;
            }
        }

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
    graph.printGraph();
    graph.floydWarshall();
    return 0;
}