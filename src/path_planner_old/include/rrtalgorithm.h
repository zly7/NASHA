#ifndef RRTALGORITHM_H
#define RRTALGORITHM_H
#include "node2d.h"
#include "collisiondetection.h"
#include "visualize.h"
#include <vector>
namespace HybridAStar {
class RrtAlgorithm {
public:
    RrtAlgorithm();
    int minX;
    int minY;
    int maxX;
    int maxY;
    Node2D* rrtStar(Node2D& start, const Node2D& goal, Node2D* nodes2D,
                    int width, int height, CollisionDetection& configurationSpace,
                    Visualize& visualization, int maxIterations);
private:
    Node2D* findBestParent(std::vector<Node2D*>& tree, const Node2D& randomNode,
                           CollisionDetection& configurationSpace);
    void rewire(std::vector<Node2D*>& tree, Node2D* newNode,
                CollisionDetection& configurationSpace);
    std::vector<Node2D*> findNearbyNodes(const std::vector<Node2D*>& tree, 
                                         const Node2D& node);
    float distance(const Node2D& nodeA, const Node2D& nodeB);
    Node2D sampleRandomNode(int width, int height);
    Node2D* findNearestNode(const std::vector<Node2D*>& tree, const Node2D& randomNode);
    Node2D* generateNewNode(const Node2D* nearestNode, const Node2D& randomNode);
    void updateExpansionRange(Node2D* newNode,int width, int height);
};
}
#endif 
