#include "rrtalgorithm.h"
#include <cstdlib>
using namespace HybridAStar;
RrtAlgorithm::RrtAlgorithm() {
}
Node2D* RrtAlgorithm::rrtStar(Node2D& start,
                              const Node2D& goal,
                              Node2D* nodes2D,
                              int width,
                              int height,
                              CollisionDetection& configurationSpace,
                              Visualize& visualization,
                              int maxIterations) {
    std::vector<Node2D*> tree;
    tree.push_back(&start);
    start.setG(0);
    minX = start.getX() - Constants::wheelBase;
    maxX = start.getX() + Constants::wheelBase;
    minY = start.getY() - Constants::wheelBase;
    maxY = start.getY() + Constants::wheelBase;
    Node2D* nearestNode;
    Node2D* newNode;
    for (int i = 0; i < maxIterations; ++i) {
        Node2D randomNode = sampleRandomNode(width, height);
        nearestNode = findBestParent(tree, randomNode, configurationSpace);
        if (nearestNode == nullptr) {
            continue;
        }
        newNode = generateNewNode(nearestNode, randomNode);
        newNode->setPred(nearestNode);
        newNode->setG(nearestNode->getG() + distance(*nearestNode, *newNode));
        if (configurationSpace.isTraversable(newNode)) {
            rewire(tree, newNode, configurationSpace);
            tree.push_back(newNode);
            if (Constants::visualization2D) {
                visualization.publishNode2DPoses(*newNode);
            }
            if (newNode->equalWithTol(goal, Constants::arcLengthForAstarSuccessor)){
                return newNode; 
            }
        }
    }
    return nullptr;
}
Node2D* RrtAlgorithm::findNearestNode(const std::vector<Node2D*>& tree, const Node2D& randomNode) {
    Node2D* nearestNode = nullptr;
    float minDistance = std::numeric_limits<float>::max();
    for (auto& node : tree) {
        float distance = std::sqrt(std::pow(node->getX() - randomNode.getX(), 2) +
                                   std::pow(node->getY() - randomNode.getY(), 2));
        if (distance < minDistance) {
            minDistance = distance;
            nearestNode = node;
        }
    }
    return nearestNode;
}
Node2D* RrtAlgorithm::generateNewNode(const Node2D* nearestNode, const Node2D& randomNode) {
    const float stepSize = Constants::arcLengthForAstarSuccessor;
    float dx = randomNode.getX() - nearestNode->getX();
    float dy = randomNode.getY() - nearestNode->getY();
    float mag = sqrt(dx * dx + dy * dy);
    dx /= mag; dy /= mag;
    int newX = nearestNode->getX() + static_cast<int>(dx * stepSize);
    int newY = nearestNode->getY() + static_cast<int>(dy * stepSize);
    return new Node2D(newX, newY);
}
Node2D* RrtAlgorithm::findBestParent(std::vector<Node2D*>& tree, const Node2D& randomNode, CollisionDetection& configurationSpace) {
    std::vector<Node2D*> nearbyNodes = findNearbyNodes(tree, randomNode);
    Node2D* bestParent = nullptr;
    float minCost = std::numeric_limits<float>::max();
    for (Node2D* potentialParent : nearbyNodes) {
        float cost = potentialParent->getG() + distance(*potentialParent, randomNode);
        if (cost < minCost) {
            bestParent = potentialParent;
            minCost = cost;
        }
    }
    return bestParent;
}
void RrtAlgorithm::rewire(std::vector<Node2D*>& tree, Node2D* newNode, CollisionDetection& configurationSpace) {
    std::vector<Node2D*> nearbyNodes = findNearbyNodes(tree, *newNode);
    for (Node2D* potentialChild : nearbyNodes) {
        float distanceGap = distance(*newNode, *potentialChild);
        if(distanceGap < Constants::arcLengthForAstarSuccessor * 1.5){
            float newCost = newNode->getG()+distanceGap;
            if (newCost < potentialChild->getG()) {
                potentialChild->setPred(newNode);
                potentialChild->setG(newCost);
            }
        }
    }
}
std::vector<Node2D*> RrtAlgorithm::findNearbyNodes(const std::vector<Node2D*>& tree, const Node2D& node) {
    std::vector<Node2D*> nearbyNodes;
    float searchRadius = Constants::wheelBase;
    for (Node2D* potentialNode : tree) {
        if (distance(node, *potentialNode) <= searchRadius) {
            nearbyNodes.push_back(potentialNode);
        }
    }
    return nearbyNodes;
}
float RrtAlgorithm::distance(const Node2D& nodeA, const Node2D& nodeB) {
    return std::sqrt(std::pow(nodeA.getX() - nodeB.getX(), 2) +
                     std::pow(nodeA.getY() - nodeB.getY(), 2));
}
Node2D RrtAlgorithm::sampleRandomNode(int width, int height) {
    int x = std::rand() % width;
    int y = std::rand() % height;
    return Node2D(x, y);
}
void RrtAlgorithm::updateExpansionRange(Node2D* newNode, int width, int height) {
    if (newNode) {
        minX = std::max(1, std::min(newNode->getX(), minX));
        maxX = std::min(width, std::max(newNode->getX(), maxX));
        minY = std::max(1, std::min(newNode->getY(), minY));
        maxY = std::min(height, std::max(newNode->getY(), maxY));
    }
}
