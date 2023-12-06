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

void smoothPath(std::vector<Node>& path) {
    // Simple path smoothing: remove consecutive nodes that are collinear
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
    path = std::move(smoothedPath);
}

// Function to generate a random obstacle map (for testing purposes)
std::vector<std::vector<bool>> generateRandomObstacleMap(int width, int height, int obstacleDensity) {
    std::vector<std::vector<bool>> obstacleMap(height, std::vector<bool>(width, false));

    for (int i = 0; i < obstacleDensity; ++i) {
        int x = rand() % width;
        int y = rand() % height;

        obstacleMap[y][x] = true;
    }

    return obstacleMap;
}

// Function to check if a node is valid in the map
bool isValidNode(const Node& node, const std::vector<std::vector<bool>>& obstacleMap) {
    return node.x >= 0 && node.x < obstacleMap[0].size() &&
        node.y >= 0 && node.y < obstacleMap.size() &&
        !obstacleMap[node.y][node.x];
}


std::vector<Node> aStar(const Node& start, const Node& end, const std::vector<std::vector<bool>>& obstacleMap) {
    std::vector<Node> path;
    std::vector<Node> openList;
    std::vector<Node> closeList;

    openList.push_back(start);

    while (!openList.empty()) {
        // Sort openList based on f values
        std::sort(openList.begin(), openList.end(), CompareNodes());

        Node currentNode = openList.front();
        openList.erase(openList.begin());

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

                for (const auto& openNode : openList) {
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
                        openList.push_back(neighbor);
                    }
                }
            }
        }
    }

    return path; // Return an empty path if no path is found
}

// Function to perform EBS-A* algorithm
std::vector<Node> ebsAStar(const Node& start, const Node& end, int numNodes, const std::vector<std::vector<bool>>& obstacleMap) {
    std::vector<Node> path;
    std::vector<Node> pathS, pathE;
    std::vector<Node> openList1, closeList1;
    std::vector<Node> openList2, closeList2;

    openList1.push_back(start);
    openList2.push_back(end);

    while (!openList1.empty() && !openList2.empty()) {
        Node nodeS = openList1.back();
        openList1.pop_back();
        pathS.push_back(nodeS);

        if (nodeS == end) {
            break;
        }

        closeList1.push_back(nodeS);

        auto neighborsS = getNeighbors(nodeS);
        for (const auto& neighbor : neighborsS) {
            if (isValidNode(neighbor, obstacleMap) &&
                std::find(closeList1.begin(), closeList1.end(), neighbor) == closeList1.end() &&
                std::find(openList1.begin(), openList1.end(), neighbor) == openList1.end()) {
                openList1.push_back(neighbor);
            }
        }

        Node nodeE = openList2.back();
        openList2.pop_back();
        pathE.push_back(nodeE);

        closeList2.push_back(nodeE);

        auto neighborsE = getNeighbors(nodeE);
        for (const auto& neighbor : neighborsE) {
            if (isValidNode(neighbor, obstacleMap) &&
                std::find(closeList2.begin(), closeList2.end(), neighbor) == closeList2.end() &&
                std::find(openList2.begin(), openList2.end(), neighbor) == openList2.end()) {
                openList2.push_back(neighbor);
            }
        }
    }

    path = pathS;
    std::reverse(pathE.begin(), pathE.end());
    path.insert(path.end(), pathE.begin(), pathE.end());

    smoothPath(path);

    return path;
}

int main() {
    srand(static_cast<unsigned>(time(0))); // Seed for random number generation

    int width = 100;
    int height = 100;
    int obstacleDensity = 50;


    int numNodes = 100;
    Node start = { 0, 0 };
    Node end = { width - 1, height - 1};

    std::vector<std::vector<bool>> obstacleMap = generateRandomObstacleMap(width, height, obstacleDensity);

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Node> pathEBS_AStar = ebsAStar(start, end, numNodes, obstacleMap);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // Print the duration in seconds
    std::cout << "EBS-A*:\n";
    std::cout << "Time taken: " << duration.count() * 1.0 / 1000  << " seconds\n\n";

    auto start_time2 = std::chrono::high_resolution_clock::now();
    std::vector<Node> pathAStar = aStar(start, end, obstacleMap);
    auto end_time2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time2 - start_time2);
    // Print the duration in seconds
    std::cout << "A*:\n";
    std::cout << "Time taken: " << duration2.count() * 1.0 / 1000 << " seconds\n\n";

    // Print the path
    // std::cout << "Path:\n";
    // for (const auto& node : pathEBS_AStar) {
    //     std::cout << "(" << node.x << ", " << node.y << ") ";
    // }

    return 0;
}
