#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <queue>
#include <set>
#include <unordered_set>

struct Node {
    int x, y;
    double g; // Cost from start to current node
    double h; // Heuristic cost from current node to goal
    double f; // Total cost (g + h)
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct Hash {
    size_t operator()(const Node& other) const {
        std::string temp = std::to_string(other.x) + "." + std::to_string(other.y);
        return std::hash<std::string>()(temp);
    }
};

struct CompareNodes {
    bool operator()(const Node& a, const Node& b) const {
        return a.f > b.f;
    }
};

double distance(const Node& a, const Node& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

std::vector<Node> getNeighbors(const Node& node) {
    std::vector<Node> neighbors{
        {node.x + 1, node.y},
        {node.x - 1, node.y},
        {node.x, node.y + 1},
        {node.x, node.y - 1}
    };

    return neighbors;
}

void smoothPath(std::vector<Node>& path) {
    if (path.size() < 3) {
        return;
    }

    std::vector<Node> smoothedPath;
    smoothedPath.push_back(path.front());

    for (size_t i = 1; i < path.size() - 1; ++i) {
        if ((path[i].x - path[i - 1].x) * (path[i + 1].y - path[i].y) !=
            (path[i + 1].x - path[i].x) * (path[i].y - path[i - 1].y)) {
            smoothedPath.push_back(path[i]);
        }
    }

    smoothedPath.push_back(path.back());
    path.swap(smoothedPath);
}

std::vector<std::vector<bool>> generateRandomObstacleMap(int width, int height, int obstacleDensity) {
    std::vector<std::vector<bool>> obstacleMap(height, std::vector<bool>(width, false));

    for (int i = 0; i < obstacleDensity; ++i) {
        int x = rand() % width;
        int y = rand() % height;

        obstacleMap[y][x] = true;
    }

    return obstacleMap;
}

bool isValidNode(const Node& node, const std::vector<std::vector<bool>>& obstacleMap) {
    return node.x >= 0 && node.x < obstacleMap[0].size() &&
        node.y >= 0 && node.y < obstacleMap.size() &&
        !obstacleMap[node.y][node.x];
}

std::vector<Node> aStar(const Node& start, const Node& end, const std::vector<std::vector<bool>>& obstacleMap) {
    std::vector<Node> path;
    std::set<Node, CompareNodes> openList;
    std::unordered_set<Node, Hash> closeList;

    openList.insert(start);

    while (!openList.empty()) {
        Node currentNode = *openList.begin();
        openList.erase(*openList.begin());

        if (currentNode == end) {
            // Reconstruct path
            while (!(currentNode == start)) {
                path.push_back(currentNode);
                auto it = closeList.find(currentNode);
                if (it == closeList.end()) {
                    break;
                }
                currentNode = *it;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closeList.insert(currentNode);

        auto neighbors = getNeighbors(currentNode);
        for (auto& neighbor : neighbors) {
            if (isValidNode(neighbor, obstacleMap) &&
                closeList.find(neighbor) == closeList.end()) {
                double tentative_g = currentNode.g + distance(currentNode, neighbor);
                bool inOpenList = false;

                for (const Node& openNode : openList) {
                    if (openNode == neighbor) {
                        inOpenList = true;
                        break;
                    }
                }

                if (!inOpenList || tentative_g < neighbor.g) {
                    neighbor.g = tentative_g;
                    neighbor.h = distance(neighbor, end);
                    neighbor.f = neighbor.g + neighbor.h;
                    if (!inOpenList) {
                        openList.insert(neighbor);
                    }
                }
            }
        }
    }

    return path; 
}

std::vector<Node> ebsAStar(const Node& start, const Node& end, int numNodes, const std::vector<std::vector<bool>>& obstacleMap) {
    std::vector<Node> pathS, pathE;

    std::unordered_set<Node, Hash> openList1, closeList1;
    std::unordered_set<Node, Hash> openList2, closeList2;

    openList1.insert(start);
    openList2.insert(end);

    while (!openList1.empty() && !openList2.empty()) {
        Node nodeS = *openList1.begin();
        openList1.erase(*openList1.begin());
        pathS.push_back(nodeS);

        if (nodeS == end) {
            break;
        }

        closeList1.insert(nodeS);

        auto neighborsS = getNeighbors(nodeS);
        for (const auto& neighbor : neighborsS) {
            if (isValidNode(neighbor, obstacleMap) &&
                closeList1.find(neighbor) == closeList1.end() &&
                openList1.find(neighbor) == openList1.end()) {
                openList1.insert(neighbor);
            }
        }

        Node nodeE = *openList2.begin();
        openList2.erase(*openList2.begin());
        pathE.push_back(nodeE);

        closeList2.insert(nodeE);

        auto neighborsE = getNeighbors(nodeE);
        for (const auto& neighbor : neighborsE) {
            if (isValidNode(neighbor, obstacleMap) &&
                closeList2.find(neighbor) == closeList2.end() &&
                openList2.find(neighbor) == openList2.end()) {
                openList2.insert(neighbor);
            }
        }
    }

    std::reverse(pathE.begin(), pathE.end());
    pathS.insert(pathS.end(), pathE.begin(), pathE.end());

    smoothPath(pathS);

    return pathS;
}

int main() {
    srand(static_cast<unsigned>(time(0))); 

    int width = 300;
    int height = 300;
    int obstacleDensity = 1000;


    int numNodes = 100;
    Node start = { 0, 0 };
    Node end = { width - 1, height - 1};

    std::vector<std::vector<bool>> obstacleMap = generateRandomObstacleMap(width, height, obstacleDensity);

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Node> pathEBS_AStar = ebsAStar(start, end, numNodes, obstacleMap);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "EBS-A*:\n";
    std::cout << "Time taken: " << duration.count() * 1.0 / 1000  << " seconds\n\n";

    auto start_time2 = std::chrono::high_resolution_clock::now();
    std::vector<Node> pathAStar = aStar(start, end, obstacleMap);
    auto end_time2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time2 - start_time2);

    std::cout << "A*:\n";
    std::cout << "Time taken: " << duration2.count() * 1.0 / 1000 << " seconds\n\n";

    return 0;
}
