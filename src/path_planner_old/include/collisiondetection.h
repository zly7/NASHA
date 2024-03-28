#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H
#include <nav_msgs/OccupancyGrid.h>
#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"
namespace HybridAStar {
namespace {
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  if(abs(node->getX()-node->getFloatX())<1e-6){
    x = node->getFloatX();
    y = node->getFloatY();
  }else{
    x = node->getX() + 0.5;
    y = node->getY() + 0.5;
  }
  t = 99;
}
inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}
class CollisionDetection {
 public:
  bool whetherVisualize = false;
  CollisionDetection();
  template<typename T> bool isTraversable(const T* node) const {
    float cost = 1;
    float x;
    float y;
    float t;
    getConfiguration(node, x, y, t);
    if(t == 99){
      for(int j = 0 ; j <Constants::headings; j ++){
        if(configurationTest(x+0.5, y+0.5, j*Constants::deltaHeadingRad+Constants::deltaHeadingRad/2)){
          return true;
        }
      }
    }else{
      cost = configurationTest(x, y, t) ? -1 : 1;
    }
    return cost <= 0;
  }
  bool isTraversablePreciseFor2D(const Node2D* node) const {
    float x = node->getFloatX();
    float y = node->getFloatY();
    for(int j = 0 ; j <Constants::headings; j ++){
      if(configurationTest(x, y, j*Constants::deltaHeadingRad+Constants::deltaHeadingRad/2)){
        return true;
      }
    }
    return false;
  }
  bool isTraversablePreciseFor2DWithTolerance(const Node2D* node) const {
    float x = node->getFloatX();
    float y = node->getFloatY();
    for(int j = 0 ; j <Constants::headings; j ++){
      if(configurationTestWithTolerace(x, y, j*Constants::deltaHeadingRad+Constants::deltaHeadingRad/2)){
        return true;
      }
    }
    return false;
  }
  bool isTraversableWithTolerance(const Node3D* node, int tolerance = -1) const {
    float x = node->getX();
    float y = node->getY();
    float t = node->getT();
    if(configurationTestWithTolerace(x, y, t, tolerance)){
      return true;
    }
    return false;
  }
  template<typename T> bool isObstacleThisPoint(const T* node) const {
      return !grid->data[node->getIdx()];
  }
  bool isObstacleWidthCircle(Node2D* node) const {
      float width = Constants::width;
      float radius = Constants::width / 2; 
      int centerX = node->getFloatX();
      int centerY = node->getFloatY();
      float diagonalRadius = radius * std::sqrt(2) / 2; 
      std::vector<std::pair<int, int>> directions = {
          {centerX + radius, centerY},                        
          {centerX - radius, centerY},                        
          {centerX, centerY + radius},                        
          {centerX, centerY - radius},                        
          {centerX + diagonalRadius, centerY + diagonalRadius}, 
          {centerX - diagonalRadius, centerY + diagonalRadius}, 
          {centerX + diagonalRadius, centerY - diagonalRadius}, 
          {centerX - diagonalRadius, centerY - diagonalRadius}  
      };
      for (const auto& direction : directions) {
          size_t index = direction.second * Node2D::widthForMap + direction.first;
          if (index < grid->data.size() && grid->data[index]) {
              return false; 
          }
      }
      return !grid->data[node->getIdx()]; 
  }
  float configurationCost(float x, float y, float t) const {return 0;}
  bool configurationTest(float x, float y, float t) const;
  bool configurationTestWithTolerace(float x, float y, float t, int tolerance = -1) const;
  void visualizeGrid(nav_msgs::OccupancyGrid::Ptr grid, int gridSize, int gridWidth, int gridHeight) const;
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;} 
  void visualizeGridAndVehicle(float x, float y, float t) const;
 private:
  nav_msgs::OccupancyGrid::Ptr grid;
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}
#endif 
