#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <algorithm>

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
        unordered_set<int> visited;
        priority_queue<Node, vector<Node>, greater<Node>> pq;

        for (auto &[node, _] : adj) {
            dist[node] = numeric_limits<double>::infinity();
        }

        dist[start] = 0;
        pq.push({start, 0});

        while (!pq.empty()) {
            int current = pq.top().id;
            pq.pop();

            if (visited.find(current) != visited.end()) continue;
            visited.insert(current);

            if (current == goal) break;

            for (const auto &edge : adj[current]) {
                double newDist = dist[current] + edge.weight;
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
        reverse(path.begin(), path.end());
        return path;
    }

    vector<int> aStar(int start, int goal, unordered_map<int, pair<int, int>> &coordinates) {
        auto heuristic = [&](int a, int b) {
            return sqrt(pow(coordinates[a].first - coordinates[b].first, 2) +
                        pow(coordinates[a].second - coordinates[b].second, 2));
        };

        unordered_map<int, int> parent;
        unordered_map<int, double> gScore, fScore;
        unordered_set<int> visited;
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

            if (visited.find(current) != visited.end()) continue;
            visited.insert(current);

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
        reverse(path.begin(), path.end());
        return path;
    }

    vector<int> bellmanFord(int start, int goal) {
        unordered_map<int, double> dist;
        unordered_map<int, int> parent;

        for (auto &[node, _] : adj) {
            dist[node] = numeric_limits<double>::infinity();
        }

        dist[start] = 0;

        for (size_t i = 0; i < adj.size() - 1; i++) {
            for (auto &[u, edges] : adj) {
                for (auto &edge : edges) {
                    if (dist[u] + edge.weight < dist[edge.to]) {
                        dist[edge.to] = dist[u] + edge.weight;
                        parent[edge.to] = u;
                    }
                }
            }
        }

        for (auto &[u, edges] : adj) {
            for (auto &edge : edges) {
                if (dist[u] + edge.weight < dist[edge.to]) {
                    cout << "Negative cycle detected!" << endl;
                    return {};
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
};

int main() {
    Graph graph;
    int edges, u, v, w;
    bool directed;
    
    cout << "Enter number of edges: ";
    cin >> edges;
    cout << "Is the graph directed? (1 for Yes, 0 for No): ";
    cin >> directed;

    cout << "Enter edges (u v w):\n";
    for (int i = 0; i < edges; i++) {
        cin >> u >> v >> w;
        graph.addEdge(u, v, w, directed);
    }

    unordered_map<int, pair<int, int>> coordinates;
    cout << "Enter node coordinates (id x y):\n";
    for (int i = 0; i < edges + 1; i++) {
        int id, x, y;
        cin >> id >> x >> y;
        coordinates[id] = {x, y};
    }

    int start, goal;
    cout << "Enter start and goal nodes: ";
    cin >> start >> goal;

    graph.printGraph();

    vector<int> pathDijkstra = graph.dijkstra(start, goal);
    vector<int> pathAStar = graph.aStar(start, goal, coordinates);
    vector<int> pathBellmanFord = graph.bellmanFord(start, goal);

    auto printPath = [](const string &name, const vector<int> &path) {
        cout << name << " Path: ";
        if (path.empty()) cout << "No path found!";
        for (int node : path) cout << node << " ";
        cout << endl;
    };

    printPath("Dijkstra", pathDijkstra);
    printPath("A*", pathAStar);
    printPath("Bellman-Ford", pathBellmanFord);

    return 0;
}