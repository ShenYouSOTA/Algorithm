#include "reference.h"
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <limits>
#include <algorithm>
#include <sstream>

namespace reference {
// Structure to represent an edge in the graph
struct Edge {
    std::string dest;
    int weight;
    
    Edge(const std::string& d, int w) : dest(d), weight(w) {}
};

// Structure to represent a graph node
struct Node {
    std::string name;
    std::vector<Edge> neighbors;

    Node() : name() {}                  // ← 新增：允许无参构造
    Node(const std::string& n) : name(n) {}
};


// Graph class
class Graph {
private:
    std::unordered_map<std::string, Node> nodes;
    int numFloors;
    
    // Helper to add an edge between two nodes
    void addEdge(const std::string& from, const std::string& to, int weight) {
        nodes[from].neighbors.push_back(Edge(to, weight));
        nodes[to].neighbors.push_back(Edge(from, weight)); // Undirected graph
    }
    
public:
    Graph(int floors) : numFloors(floors) {
        buildGraph();
    }
    
    // Parse node type and details
    std::tuple<char, int, int, std::string> parseNode(const std::string& nodeName) {
        char tower = nodeName[0]; // A or B
        std::string nodeType;
        int floor, index;
        
        if (nodeName.length() >= 3 && (nodeName[1] == 'C' || nodeName[1] == 'E')) {
            // Corridor or elevator node
            nodeType = nodeName.substr(1, 1);
            floor = std::stoi(nodeName.substr(2, nodeName.length() - 4));
            index = std::stoi(nodeName.substr(nodeName.length() - 2));
        } else {
            // Room node
            nodeType = "R"; // Room
            floor = std::stoi(nodeName.substr(1, nodeName.length() - 3));
            index = std::stoi(nodeName.substr(nodeName.length() - 2));
        }
        
        return std::make_tuple(tower, floor, index, nodeType);
    }
    
    // Build the entire graph structure
    void buildGraph() {
        // Build all nodes first
        buildNodes();
        
        // Connect all nodes according to the rules
        buildInternalConnections();
    }
    
    // Create all nodes in the graph
    void buildNodes() {
        // Create nodes for all floors
        for (int floor = 1; floor <= numFloors; floor++) {
            // Tower A Rooms (A101-A114 for floor 1, A201-A214 for floor 2, etc.)
            for (int i = 1; i <= 14; i++) {
                std::string roomName = "A" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                nodes.emplace(roomName, Node(roomName));
            }
            
            // Tower A Rooms (A115-A128 for floor 1, A215-A228 for floor 2, etc.)
            for (int i = 15; i <= 28; i++) {
                std::string roomName = "A" + std::to_string(floor) + std::to_string(i);
                nodes.emplace(roomName, Node(roomName));
            }
            
            // Tower B Rooms (B101-B114 for floor 1, B201-B214 for floor 2, etc.)
            for (int i = 1; i <= 14; i++) {
                std::string roomName = "B" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                nodes.emplace(roomName, Node(roomName));
            }
            
            // Tower B Rooms (B115-B128 for floor 1, B215-B228 for floor 2, etc.)
            for (int i = 15; i <= 28; i++) {
                std::string roomName = "B" + std::to_string(floor) + std::to_string(i);
                nodes.emplace(roomName, Node(roomName));
            }
            
            // Tower A Corridors (AC101-AC114 for floor 1, AC201-AC214 for floor 2, etc.)
            for (int i = 1; i <= 14; i++) {
                std::string corridorName = "AC" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                nodes.emplace(corridorName, Node(corridorName));
            }
            
            // Tower B Corridors (BC101-BC114 for floor 1, BC201-BC214 for floor 2, etc.)
            for (int i = 1; i <= 14; i++) {
                std::string corridorName = "BC" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                nodes.emplace(corridorName, Node(corridorName));
            }
            
            // Tower A Elevators (AE101-AE102 for floor 1, AE201-AE202 for floor 2, etc.)
            for (int i = 1; i <= 2; i++) {
                std::string elevatorName = "AE" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                nodes.emplace(elevatorName, Node(elevatorName));
            }
            
            // Tower B Elevators (BE101-BE102 for floor 1, BE201-BE202 for floor 2, etc.)
            for (int i = 1; i <= 2; i++) {
                std::string elevatorName = "BE" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                nodes.emplace(elevatorName, Node(elevatorName));
            }
        }
    }
    
    // Connect nodes to form the graph structure
    void buildInternalConnections() {
        // Connect rooms to corridors and corridors to each other (5 weight)
        for (int floor = 1; floor <= numFloors; floor++) {
            // Connect Tower A rooms to corridors
            for (int i = 1; i <= 14; i++) {
                std::string roomName = "A" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                std::string corridorName = "AC" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                addEdge(roomName, corridorName, 5);
            }
            
            for (int i = 15; i <= 28; i++) {
                std::string roomName = "A" + std::to_string(floor) + std::to_string(i);
                std::string corridorName = "AC" + std::to_string(floor) + (i-14 < 10 ? "0" : "") + std::to_string(i-14);
                addEdge(roomName, corridorName, 5);
            }
            
            // Connect Tower B rooms to corridors
            for (int i = 1; i <= 14; i++) {
                std::string roomName = "B" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                std::string corridorName = "BC" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                addEdge(roomName, corridorName, 5);
            }
            
            for (int i = 15; i <= 28; i++) {
                std::string roomName = "B" + std::to_string(floor) + std::to_string(i);
                std::string corridorName = "BC" + std::to_string(floor) + (i-14 < 10 ? "0" : "") + std::to_string(i-14);
                addEdge(roomName, corridorName, 5);
            }
            
            // Connect Tower A corridors to each other
            for (int i = 1; i < 14; i++) {
                std::string corridor1 = "AC" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                std::string corridor2 = "AC" + std::to_string(floor) + (i+1 < 10 ? "0" : "") + std::to_string(i+1);
                addEdge(corridor1, corridor2, 5);
            }
            
            // Connect Tower B corridors to each other
            for (int i = 1; i < 14; i++) {
                std::string corridor1 = "BC" + std::to_string(floor) + (i < 10 ? "0" : "") + std::to_string(i);
                std::string corridor2 = "BC" + std::to_string(floor) + (i+1 < 10 ? "0" : "") + std::to_string(i+1);
                addEdge(corridor1, corridor2, 5);
            }
            
            // Connect elevators to corridors (8 weight)
            // Tower A elevators
            std::string ae1 = "AE" + std::to_string(floor) + "01";
            std::string ae2 = "AE" + std::to_string(floor) + "02";
            std::string ac1 = "AC" + std::to_string(floor) + "01";
            std::string ac14 = "AC" + std::to_string(floor) + "14";
            
            addEdge(ae1, ac1, 8);
            addEdge(ae2, ac14, 8);
            
            // Tower B elevators
            std::string be1 = "BE" + std::to_string(floor) + "01";
            std::string be2 = "BE" + std::to_string(floor) + "02";
            std::string bc1 = "BC" + std::to_string(floor) + "01";
            std::string bc14 = "BC" + std::to_string(floor) + "14";
            
            addEdge(be1, bc1, 8);
            addEdge(be2, bc14, 8);
        }
        
        // Connect ground floor towers (weight 100)
        addEdge("AE101", "BE101", 100);
        addEdge("AE102", "BE102", 100);
        

    }
    
    // Add elevator edges with weights from the eWeights matrix
    void connectElevators(int **eWeights) {
        for (int floor = 1; floor < numFloors; floor++) {
            // Tower A elevators
            std::string ae1_curr = "AE" + std::to_string(floor) + "01";
            std::string ae1_next = "AE" + std::to_string(floor+1) + "01";
            addEdge(ae1_curr, ae1_next, eWeights[0][floor-1]);
            
            std::string ae2_curr = "AE" + std::to_string(floor) + "02";
            std::string ae2_next = "AE" + std::to_string(floor+1) + "02";
            addEdge(ae2_curr, ae2_next, eWeights[1][floor-1]);
            
            // Tower B elevators
            std::string be1_curr = "BE" + std::to_string(floor) + "01";
            std::string be1_next = "BE" + std::to_string(floor+1) + "01";
            addEdge(be1_curr, be1_next, eWeights[2][floor-1]);
            
            std::string be2_curr = "BE" + std::to_string(floor) + "02";
            std::string be2_next = "BE" + std::to_string(floor+1) + "02";
            addEdge(be2_curr, be2_next, eWeights[3][floor-1]);
        }
    }

    // Add cross-tower connections on every 10th floor
    void connectCrossBeams(int **eWeights) {
        for (int floor = 10; floor <= numFloors; floor += 10) {
            // A{n}22 ↔ B{n–1}08
            if (floor > 1) {
                std::string a_node = "A" + std::to_string(floor) + "22";
                std::string b_node = "B" + std::to_string(floor-1) + "08";
                addEdge(a_node, b_node, eWeights[0][0]); // Same weight as 1st elevator
            }
            
            // A{n}23 ↔ B{n+1}09
            if (floor < numFloors) {
                std::string a_node = "A" + std::to_string(floor) + "23";
                std::string b_node = "B" + std::to_string(floor+1) + "09";
                addEdge(a_node, b_node, eWeights[0][0]); // Same weight as 1st elevator
            }
        }
    }
    
    // Dijkstra's algorithm to find shortest path
    int dijkstra(const std::string& start, const std::string& end, std::vector<std::string>& path) {
        // Priority queue for Dijkstra's algorithm
        // Pair: (distance, node_name)
        using pq_element = std::pair<int, std::string>;
        std::priority_queue<pq_element, std::vector<pq_element>, std::greater<pq_element>> pq;
        
        // Map to store distances
        std::unordered_map<std::string, int> dist;
        
        // Map to store predecessors for path reconstruction
        std::unordered_map<std::string, std::string> prev;
        
        // Initialize distances to infinity
        for (const auto& node_pair : nodes) {
            dist[node_pair.first] = std::numeric_limits<int>::max();
        }
        
        // Distance to start is 0
        dist[start] = 0;
        pq.push({0, start});
        
        // Run Dijkstra's algorithm
        while (!pq.empty()) {
            std::string u = pq.top().second;
            int d = pq.top().first;
            pq.pop();
            
            // Skip if we've already found a shorter path
            if (d > dist[u]) continue;
            
            // Stop if we reached the destination
            if (u == end) break;
            
            // Check all neighbors of u
            for (const Edge& edge : nodes[u].neighbors) {
                std::string v = edge.dest;
                int w = edge.weight;
                
                // If we found a shorter path to v through u
                if (dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
        
        // Reconstruct the path
        path.clear();
        if (dist[end] != std::numeric_limits<int>::max()) {
            // Path exists, reconstruct it
            std::string curr = end;
            while (curr != start) {
                path.push_back(curr);
                curr = prev[curr];
            }
            path.push_back(start);
            
            // Reverse to get path from start to end
            std::reverse(path.begin(), path.end());
        }
        
        return dist[end] == std::numeric_limits<int>::max() ? -1 : dist[end];
    }
    
    int getNumFloors() const {
        return numFloors;
    }
};

// Function to find the shortest path
int FindShortestPath(const std::string &start, const std::string &end, 
                     std::vector<std::string> &path, int **eWeights) {
    // Determine number of floors by analyzing the eWeights array
    // eWeights is 4xm where m is the number of floors
    // We need to infer the number of floors from the context or arguments
    
    // For this implementation, let's extract the higher floor from start or end
    int maxFloor = 1;
    
    // Extract floor from start
    if (start.length() >= 3) {
        int startFloor;
        if (start[1] == 'C' || start[1] == 'E') {
            startFloor = std::stoi(start.substr(2, start.length() - 4));
        } else {
            startFloor = std::stoi(start.substr(1, start.length() - 3));
        }
        maxFloor = std::max(maxFloor, startFloor);
    }
    
    // Extract floor from end
    if (end.length() >= 3) {
        int endFloor;
        if (end[1] == 'C' || end[1] == 'E') {
            endFloor = std::stoi(end.substr(2, end.length() - 4));
        } else {
            endFloor = std::stoi(end.substr(1, end.length() - 3));
        }
        maxFloor = std::max(maxFloor, endFloor);
    }
    
    // Add some extra floors to be safe
    // In a real implementation, this should be passed as an argument
    int numFloors = maxFloor + 10;
    
    // Build the graph
    Graph graph(numFloors);
    
    // Connect elevators using the provided weights
    graph.connectElevators(eWeights);

    // Connect crossbeams
    graph.connectCrossBeams(eWeights);
    
    // Run Dijkstra's algorithm to find the shortest path
    return graph.dijkstra(start, end, path);
}
}
