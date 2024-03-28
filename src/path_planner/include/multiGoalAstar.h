#ifndef MULTIGOALASTAR_H
#define MULTIGOALASTAR_H
#include <set>
#include <vector>
#include "node3d.h"
#include <numeric>  
#include <algorithm>  
#include "collisiondetection.h"
namespace HybridAStar {
class multiGoalSet3D {
 public:
    std::vector<Node3D> goals;
    Node3D virtualCenterNode3D;
    multiGoalSet3D();
    void addGoal(const Node3D& node);
    void addGoals(const std::set<Node3D>& nodeset);
    void addGoals(const std::vector<Node3D>& nodes);
    static multiGoalSet3D fuzzyOneNodeToSet(const CollisionDetection & collectionDection,const Node3D& node);
    static multiGoalSet3D fuzzyOneNodeToSetForSplitAstar(const CollisionDetection & collectionDection,const Node3D& node); 
    Node3D getRandomGoal();
    static void printMultiGoalsBou(const std::vector<multiGoalSet3D>& multiGoalsBou);
private:
    void updateVirtualCenterNode();
};
}  
#endif 
