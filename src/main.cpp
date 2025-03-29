#include <iostream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <fstream>

using namespace std;

// Structure to represent an edge in the graph
struct Edge {
    string to;       // Destination node
    double weight;   // Weight of the edge
};

class Graph {
public:
    unordered_map<string, vector<Edge>> adj; // Adjacency list to store graph edges
    unordered_map<string, int> nodeIndex;    // Mapping node names to indices
    vector<string> nodeNames;                // List to store node names

    // Function to add an edge to the graph
    void addEdge(const string &u, const string &v, double w, bool directed = false) {
        adj[u].push_back({v, w});  // Add edge from u to v
        if (!directed) {           // If the graph is undirected
            adj[v].push_back({u, w}); // Add the reverse edge from v to u
        }

        // Add nodes to the index list if they are not already present
        if (nodeIndex.find(u) == nodeIndex.end()) {
            nodeIndex[u] = nodeNames.size();
            nodeNames.push_back(u);
        }
        if (nodeIndex.find(v) == nodeIndex.end()) {
            nodeIndex[v] = nodeNames.size();
            nodeNames.push_back(v);
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

    // Floyd-Warshall algorithm with path reconstruction
    void floydWarshall() {
        int n = nodeNames.size();
        vector<vector<double>> dist(n, vector<double>(n, numeric_limits<double>::infinity()));
        vector<vector<int>> next(n, vector<int>(n, -1)); // Table for path reconstruction

        // Initialize the distance matrix
        for (int i = 0; i < n; i++) {
            dist[i][i] = 0; // Distance from a node to itself is zero
        }

        // Initialize distances for direct edges
        for (auto &[u, edges] : adj) {
            int uIdx = nodeIndex[u];
            for (auto &edge : edges) {
                int vIdx = nodeIndex[edge.to];
                dist[uIdx][vIdx] = edge.weight;
                next[uIdx][vIdx] = vIdx; // Set initial path
            }
        }

        // Execute the Floyd-Warshall algorithm
        for (int k = 0; k < n; k++) {   // Intermediate node
            for (int i = 0; i < n; i++) {   // Source node
                for (int j = 0; j < n; j++) {   // Destination node
                    if (dist[i][k] != numeric_limits<double>::infinity() &&
                        dist[k][j] != numeric_limits<double>::infinity() &&
                        dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        next[i][j] = next[i][k]; // Update the optimal path
                    }
                }
            }
        }

        // Detect negative cycles
        for (int i = 0; i < n; i++) {
            if (dist[i][i] < 0) {
                cout << "Negative cycle detected!\n";
                return;
            }
        }

        // Print the shortest distances matrix
        cout << "Shortest distances:\n";
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                cout << (dist[i][j] == numeric_limits<double>::infinity() ? "INF" : to_string(dist[i][j])) << " ";
            }
            cout << endl;
        }
    }

    // Function to reconstruct the shortest path between two nodes
    vector<string> reconstructPath(const string &start, const string &goal) {
        if (nodeIndex.find(start) == nodeIndex.end() || nodeIndex.find(goal) == nodeIndex.end()) {
            return {}; // If either node doesn't exist, return an empty path
        }

        int u = nodeIndex[start], v = nodeIndex[goal];
        if (next[u][v] == -1) return {}; // If no path exists, return empty

        vector<string> path = {start};
        while (u != v) {
            u = next[u][v];
            if (u == -1) return {}; // Double-check for a valid path
            path.push_back(nodeNames[u]);
        }
        return path;
    }
};

int main() {
    Graph graph;
    ifstream inputFile("graph_input.txt"); // Open input file

    if (!inputFile) { // Check if the file opened correctly
        cout << "Error opening file!" << endl;
        return 1;
    }

    bool directed;
    inputFile >> directed; // Read whether the graph is directed

    string u, v;
    double w;
    while (inputFile >> u >> v >> w) {
        graph.addEdge(u, v, w, directed); // Add edges to the graph
    }
    inputFile.close(); // Close the input file

    // Print the graph structure
    graph.printGraph();

    // Run the Floyd-Warshall algorithm
    graph.floydWarshall();

    // Take user input for path finding
    string start, goal;
    cout << "Enter start and goal nodes: ";
    cin >> start >> goal;

    // Reconstruct the shortest path from start to goal
    vector<string> path = graph.reconstructPath(start, goal);

    // Print the found path
    cout << "Shortest path: ";
    if (path.empty()) cout << "No path found!";
    for (const string &node : path) cout << node << " ";
    cout << endl;

    return 0;
}