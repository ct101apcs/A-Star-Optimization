#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <queue>

struct Node {
    int x, y;
    double g; // Cost from start to current node
    double h; // Heuristic cost from current node to goal
    double f; // Total cost (g + h)
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct CompareNodes {
    bool operator()(const Node& a, const Node& b) {
        return a.f > b.f;
    }
};

double distance(const Node& a, const Node& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

std::vector<Node> getNeighbors(const Node& node) {
    // Simple grid-based neighbors (up, down, left, right)
    std::vector<Node> neighbors{
        {node.x + 1, node.y},
        {node.x - 1, node.y},
        {node.x, node.y + 1},
        {node.x, node.y - 1}
    };

    // You might need to filter out neighbors that are outside the map or blocked by obstacles.
    return neighbors;
}

bool isValidNode(const Node& node, const std::vector<std::vector<bool>>& obstacleMap) {
    return node.x >= 0 && node.x < obstacleMap[0].size() &&
        node.y >= 0 && node.y < obstacleMap.size() &&
        !obstacleMap[node.y][node.x];
}

std::vector<Node> aStar(const Node& start, const Node& end, const std::vector<std::vector<bool>>& obstacleMap) {
    std::vector<Node> path;

    std::priority_queue<Node, std::vector<Node>, CompareNodes> openList;
    std::vector<Node> closeList;

    openList.push(start);

    while (!openList.empty()) {
        Node currentNode = openList.top();
        openList.pop();

        if (currentNode == end) {
            // Reconstruct path
            while (!(currentNode == start)) {
                path.push_back(currentNode);
                auto it = std::find_if(closeList.begin(), closeList.end(),
                    [currentNode](const Node& n) { return n == currentNode; });
                if (it == closeList.end()) {
                    // Handle an unexpected situation
                    break;
                }
                currentNode = *it;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closeList.push_back(currentNode);

        auto neighbors = getNeighbors(currentNode);
        for (auto& neighbor : neighbors) {
            if (isValidNode(neighbor, obstacleMap) &&
                std::find(closeList.begin(), closeList.end(), neighbor) == closeList.end()) {
                double tentative_g = currentNode.g + distance(currentNode, neighbor);
                bool inOpenList = false;
                /*
                for (auto& openNode : openList) {
                    if (openNode == neighbor) {
                        inOpenList = true;
                        break;
                    }
                }
                */
                while (!openList.empty()) {
                    const auto openNode = openList.top();
                    openList.pop();
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
                        openList.push(neighbor);
                    }
                }
            }
        }
    }

    return path; // Return an empty path if no path is found
}