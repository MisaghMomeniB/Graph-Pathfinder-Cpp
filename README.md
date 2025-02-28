# Graph Algorithms in C++ 📊

This repository contains a C++ implementation of **Graph Algorithms**, focusing on **Dijkstra's Algorithm** (for shortest path) and **Floyd-Warshall Algorithm** (for all-pairs shortest paths). The code is designed to handle both directed and undirected graphs, allowing you to find optimal paths between nodes efficiently. 🚀

---

## Table of Contents 📚

1. [Overview](#overview)
2. [Features](#features)
3. [How to Run](#how-to-run)
4. [Explanation of Code](#explanation-of-code)
5. [Example Input/Output](#example-inputoutput)
6. [License](#license)

---

## Overview 📝

This C++ program provides an implementation of two essential graph algorithms:

1. **Dijkstra's Algorithm**: 
   - Used for finding the shortest path from a source node to a target node in a weighted graph.
   
2. **Floyd-Warshall Algorithm**:
   - Computes the shortest path between all pairs of nodes in a graph. It works for both directed and undirected graphs.

---

## Features ✨

- **Graph Representation**: 
  - Uses an **Adjacency List** to represent the graph. 
  - Supports both **directed** and **undirected** graphs.

- **Dijkstra's Algorithm**:
  - Finds the shortest path between a source and target node.
  - Uses a **priority queue** to ensure efficient pathfinding.

- **Floyd-Warshall Algorithm**:
  - Computes the shortest paths between all pairs of nodes in the graph.
  - Prints the results in a matrix form.

- **File Input**: 
  - Graph data is read from a text file (`graph_input.txt`), making it easier to work with large graphs.

---

## How to Run 🏃‍♀️

### Prerequisites

Ensure that you have a C++ compiler installed (e.g., GCC, Clang).

### Steps:

1. Clone this repository to your local machine.
2. Create an input file `graph_input.txt` containing the graph data. Example format:
   ```
   5 6 0   # 5 nodes, 6 edges, undirected graph
   0 1 10
   0 4 20
   1 2 10
   2 4 50
   2 3 30
   3 4 10
   ```

3. Compile the program:
   ```bash
   g++ graph_algorithms.cpp -o graph_algorithms
   ```

4. Run the program:
   ```bash
   ./graph_algorithms
   ```

5. Enter the **start** and **goal** nodes when prompted.

---

## Explanation of Code 💡

### Graph Class:

The `Graph` class contains the main logic of the graph algorithms:

- **Data Structure**: 
  - An `unordered_map` is used to represent the adjacency list, where each key is a node, and its value is a vector of `Edge` structs containing destination nodes and edge weights.

### Dijkstra’s Algorithm:

- The **Dijkstra** function implements Dijkstra’s shortest path algorithm using a **set** as a priority queue (min-heap).
- For each node, the algorithm keeps track of the shortest known distance from the start node. It updates these distances as it explores the graph.
- If the algorithm finds a shorter path, it updates the distance and the parent node for path reconstruction.

### Floyd-Warshall Algorithm:

- The **Floyd-Warshall** function computes the shortest path between all pairs of nodes using a dynamic programming approach.
- A matrix is used to store the shortest distances between all pairs, and it is updated through three nested loops, each representing a possible intermediate node.

### Main Function:

- The program reads graph data from a file (`graph_input.txt`), builds the graph, and allows the user to input **start** and **goal** nodes for Dijkstra’s Algorithm.
- The shortest path is computed and printed.
- The Floyd-Warshall algorithm is executed to display the shortest distances between all pairs of nodes.

---

## Example Input/Output 📂

### Example Input:

Given the following input file (`graph_input.txt`):

```
5 6 0
0 1 10
0 4 20
1 2 10
2 4 50
2 3 30
3 4 10
```

And the user inputs:

```
Start Node: 0
Goal Node: 4
```

### Example Output:

```
Graph adjacency list:
Node 0: (1, 10) (4, 20) 
Node 1: (0, 10) (2, 10) 
Node 2: (1, 10) (4, 50) (3, 30) 
Node 3: (2, 30) (4, 10) 
Node 4: (0, 20) (2, 50) (3, 10) 

Dijkstra Path: 0 1 2 3 4 

Shortest distances between every pair of vertices:
0 10 20 50 20 
10 0 10 30 20 
20 10 0 30 30 
50 30 30 0 10 
20 20 30 10 0 
```

---

Feel free to contribute, open issues, or ask questions! 😊
