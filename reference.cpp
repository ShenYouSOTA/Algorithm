#include "reference.h"
#include <queue>
#include <limits>
#include <sstream>
#include <algorithm>
#include <iomanip>

namespace reference {

struct Edge {
    int to;
    int weight;
};

int FindShortestPath(std::string start, std::string end, 
                     std::vector<std::string>& path, int** eWeights) {
    const int COST_OFFICE_CORRIDOR = 5;
    const int COST_CORRIDOR_EDGE = 5;
    const int COST_ELEVATOR_CORRIDOR = 8;
    const int COST_INTERTOWER = 100;
    const int NODES_PER_FLOOR = 44;
    const int OFFICE_COUNT = 28;
    const int CORRIDOR_COUNT = 14;
    const int MAX_FLOORS = 1000;
    
    // 快速解析办公室标签
    auto parseOfficeLabel = [](const std::string &label) {
        return std::make_pair(
            std::stoi(label.substr(1, label.size()-3)),
            std::stoi(label.substr(label.size()-2, 2))
        );
    };

    // 确定实际需要的楼层数
    auto [startFloor, startRoom] = parseOfficeLabel(start);
    auto [endFloor, endRoom] = parseOfficeLabel(end);
    int numFloors = std::max({startFloor, endFloor, 100}); // 至少100层

    // 计算总节点数并初始化图
    int totalNodes = 2 * numFloors * NODES_PER_FLOOR;
    std::vector<std::vector<Edge>> graph(totalNodes);

    // 获取节点索引的lambda函数
    auto getIndex = [=](int tower, int floor, int localIndex) {
        return (tower * numFloors + (floor - 1)) * NODES_PER_FLOOR + localIndex;
    };

    // 构建楼层内部连接
    for (int t = 0; t < 2; t++) {
        for (int f = 1; f <= numFloors; f++) {
            // 办公室到走廊的连接
            for (int r = 1; r <= OFFICE_COUNT; r++) {
                int officeNode = getIndex(t, f, r - 1);
                int door = (r <= 14 ? r : r - 14);
                int corridorNode = getIndex(t, f, OFFICE_COUNT + (door - 1));
                graph[officeNode].push_back({corridorNode, COST_OFFICE_CORRIDOR});
                graph[corridorNode].push_back({officeNode, COST_OFFICE_CORRIDOR});
            }

            // 走廊之间的连接
            for (int j = 1; j < CORRIDOR_COUNT; j++) {
                int node1 = getIndex(t, f, OFFICE_COUNT + (j - 1));
                int node2 = getIndex(t, f, OFFICE_COUNT + j);
                graph[node1].push_back({node2, COST_CORRIDOR_EDGE});
                graph[node2].push_back({node1, COST_CORRIDOR_EDGE});
            }

            // 电梯到走廊的连接
            int elev1 = getIndex(t, f, 42);
            int corr1 = getIndex(t, f, OFFICE_COUNT);
            graph[elev1].push_back({corr1, COST_ELEVATOR_CORRIDOR});
            graph[corr1].push_back({elev1, COST_ELEVATOR_CORRIDOR});

            int elev2 = getIndex(t, f, 43);
            int corr14 = getIndex(t, f, OFFICE_COUNT + 13);
            graph[elev2].push_back({corr14, COST_ELEVATOR_CORRIDOR});
            graph[corr14].push_back({elev2, COST_ELEVATOR_CORRIDOR});
        }
    }

    // 构建电梯垂直连接
    for (int t = 0; t < 2; t++) {
        for (int f = 1; f < numFloors; f++) {
            // 电梯1
            int nodeCur = getIndex(t, f, 42);
            int nodeNext = getIndex(t, f+1, 42);
            int weight = (t == 0) ? eWeights[0][f-1] : eWeights[2][f-1];
            graph[nodeCur].push_back({nodeNext, weight});
            graph[nodeNext].push_back({nodeCur, weight});

            // 电梯2
            nodeCur = getIndex(t, f, 43);
            nodeNext = getIndex(t, f+1, 43);
            weight = (t == 0) ? eWeights[1][f-1] : eWeights[3][f-1];
            graph[nodeCur].push_back({nodeNext, weight});
            graph[nodeNext].push_back({nodeCur, weight});
        }
    }

    // 构建天桥连接
    int beamWeight = eWeights[0][0];
    for (int f = 10; f <= numFloors; f += 10) {
        if (f > 1) {
            int nodeA = getIndex(0, f, 21);
            int nodeB = getIndex(1, f-1, 7);
            graph[nodeA].push_back({nodeB, beamWeight});
            graph[nodeB].push_back({nodeA, beamWeight});
        }
        if (f < numFloors) {
            int nodeA = getIndex(0, f, 22);
            int nodeB = getIndex(1, f+1, 8);
            graph[nodeA].push_back({nodeB, beamWeight});
            graph[nodeB].push_back({nodeA, beamWeight});
        }
    }

    // 一楼大楼间连接
    int nodeA_floor1 = getIndex(0, 1, 43);
    int nodeB_floor1 = getIndex(1, 1, 43);
    graph[nodeA_floor1].push_back({nodeB_floor1, COST_INTERTOWER});
    graph[nodeB_floor1].push_back({nodeA_floor1, COST_INTERTOWER});

    // 解析起点和终点
    auto parseOffice = [&](const std::string &label) {
        char tower = label[0];
        int floor = std::stoi(label.substr(1, label.size()-3));
        int room = std::stoi(label.substr(label.size()-2, 2));
        return getIndex(tower == 'A' ? 0 : 1, floor, room - 1);
    };

    int startIndex = parseOffice(start);
    int endIndex = parseOffice(end);

    // Dijkstra算法
    const int INF = std::numeric_limits<int>::max();
    std::vector<int> dist(totalNodes, INF);
    std::vector<int> prev(totalNodes, -1);
    std::priority_queue<std::pair<int, int>, 
                       std::vector<std::pair<int, int>>, 
                       std::greater<>> pq;
    
    dist[startIndex] = 0;
    pq.push({0, startIndex});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d != dist[u]) continue;
        if (u == endIndex) break;

        for (const auto &edge : graph[u]) {
            int v = edge.to;
            int nd = d + edge.weight;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (dist[endIndex] == INF) return -1;

    // 重建路径
    std::vector<int> nodePath;
    for (int cur = endIndex; cur != -1; cur = prev[cur]) {
        nodePath.push_back(cur);
    }
    std::reverse(nodePath.begin(), nodePath.end());

    // 生成路径标签
    auto nodeLabel = [&](int idx) {
        int t = idx / (numFloors * NODES_PER_FLOOR);
        int rem = idx % (numFloors * NODES_PER_FLOOR);
        int f = rem / NODES_PER_FLOOR + 1;
        int local = rem % NODES_PER_FLOOR;
        char tower = (t == 0 ? 'A' : 'B');
        std::ostringstream oss;
        
        if (local < OFFICE_COUNT) {
            oss << tower << f << std::setw(2) << std::setfill('0') << (local+1);
        } else if (local < OFFICE_COUNT + CORRIDOR_COUNT) {
            int corr = local - OFFICE_COUNT + 1;
            oss << tower << "C" << f << std::setw(2) << std::setfill('0') << corr;
        } else {
            int elevId = local - (OFFICE_COUNT + CORRIDOR_COUNT) + 1;
            oss << tower << "E" << f << std::setw(2) << std::setfill('0') << elevId;
        }
        return oss.str();
    };

    path.clear();
    for (int idx : nodePath) {
        path.push_back(nodeLabel(idx));
    }

    return dist[endIndex];
}

} // namespace reference
