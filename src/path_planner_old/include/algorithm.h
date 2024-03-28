#ifndef ALGORITHM_H
#define ALGORITHM_H
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
typedef ompl::base::SE2StateSpace::StateType State;
#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"
#include <cmath>
#include "multiGoalAstar.h"
#include <chrono>
#include <boost/heap/binomial_heap.hpp>
#include <CGAL/Circular_kernel_2.h>
#include <CGAL/Circular_arc_2.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include <cmath>
#include <directionVector.h>  
namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;
class Algorithm {
 public:
  Algorithm() {}
  static Node3D* hybridAStar(Node3D& start,
                             const Node3D& goal,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                             Visualize& visualization,Tolerance tol);
   static Node3D* hybridAStarMultiGoals(Node3D& start,
                             multiGoalSet3D& goals,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                             Visualize& visualization,Tolerance tol);              
   static Node2D* aStar2D(Node2D& start,
                              const Node2D& goal,
                              Node2D* nodes2D,
                              int width,
                              int height,
                              CollisionDetection& configurationSpace,
                              Visualize& visualization);
   static Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);
   static Node3D* ArcShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);
};
}
#endif 
