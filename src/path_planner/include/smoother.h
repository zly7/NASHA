#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "dynamicvoronoi.h"
#include "node3d.h"
#include "node2d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
namespace HybridAStar {

class Smoother {
 public:
  Smoother() {}

  
  void smoothPath(DynamicVoronoi& voronoi);

  
  void tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());
  void tracePathAndReverse(const Node3D* node);
  void tracePath2D(const Node2D* node, int i = 0, std::vector<Node2D> path = std::vector<Node2D>());

  
  const std::vector<Node3D>& getPath() {return path;}
  std::vector<Node3D>& getPathNotConst() {return path;}
  const std::vector<Node2D>& getPath2D() {return path2D;}

  
  Vector2D obstacleTerm(Vector2D xi);

  
  Vector2D curvatureTerm(Vector2D x_im2, Vector2D x_im1, Vector2D x_i, Vector2D x_ip1, Vector2D x_ip2);

  
  Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

  
  

  
  bool isOnGrid(Vector2D vec) {
    if (vec.getX() >= 0 && vec.getX() < width &&
        vec.getY() >= 0 && vec.getY() < height) {
      return true;
    }
    return false;
  }
  void clear();

 private:
  
  float kappaMax = 1.f / (Constants::r * 1.1);
  
  float obsDMax = Constants::minRoadWidth;
  
  float vorObsDMax = Constants::minRoadWidth;
  
  float alpha = 0.1;
  
  float wObstacle = 0.2;
  
  float wVoronoi = 0;
  
  float wCurvature = 0.1;
  
  float wSmoothness = 0.2;
  
  DynamicVoronoi voronoi;
  
  int width;
  
  int height;
  
  std::vector<Node3D> path;
  
  std::vector<Node2D> path2D;
};
}
#endif 
