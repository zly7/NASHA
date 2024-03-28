#ifndef NODE3D_H
#define NODE3D_H
#include <cmath>
#include <random>
#include "constants.h"
#include "helper.h"
#include <iostream>
namespace HybridAStar {
  class Tolerance {
  public:
    float distanceTolerance;
    float angelTolerance;
    Tolerance(float distanceTolerance, float angelTolerance) : distanceTolerance(distanceTolerance), angelTolerance(angelTolerance) {}
    Tolerance() : distanceTolerance(Constants::tolerance), angelTolerance(Constants::deltaHeadingRad) {}
  };
class Node3D {
 public:
  Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}
  Node3D(float x,float y, float t): Node3D(x, y, t, 0, 0, nullptr) {}
  Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = std::min((int)(t / Constants::deltaHeadingRad),Constants::headings-1) + (int)(y) * Constants::headings + (int)(x)* Constants::headings * Node3D::heightForMap;
    this->prim = prim;
  }
  float getX() const { return x; }
  float getY() const { return y; }
  float getT() const { return t; }
  float getG() const { return g; }
  float getH() const { return h; }
  float getC() const { return g + Constants::heuristicDecayCoefficient * h; }
  int getIdx() const { return idx; }
  int getPrim() const { return prim; }
  bool isOpen() const { return o; }
  bool isClosed() const { return c; }
  const Node3D* getPred() const { return pred; }
  void setX(const float& x) { this->x = x; }
  void setY(const float& y) { this->y = y; }
  void setT(const float& t) { 
    this->t = t; 
  }
  void setG(const float& g) { this->g = g; }
  void setH(const float& h) { this->h = h; }
  void setPrim(int prim) { this->prim = prim; }
  void open() { o = true; c = false;}
  void close() { c = true; o = false; }
  void setPred(const Node3D* pred) { this->pred = pred; }
  void updateG();
  bool operator == (const Node3D& rhs) const;
  bool isEqualWithTolerance (const Node3D& rhs,Tolerance tol) const;
  bool isInRange(const Node3D& goal) const;
  bool isInArcRange(const Node3D& goal) const;
  bool isOnGrid(const int width, const int height) const;
  bool isOnGrid() const;
  Node3D* createSuccessor(const int i);
  float get2DDistance(Node3D rhs){
    return sqrt((x-rhs.getX())*(x-rhs.getX())+(y-rhs.getY())*(y-rhs.getY()));
  }
  static const int dir;
  static const float arcLength;
  static const float steeringAngle;
  static std::vector<float> dx;
  static std::vector<float> dy;
  static std::vector<float> dt;
  static const float resolution_mutiplier;
  static std::mt19937 gen; 
  static std::uniform_real_distribution<> dis; 
  static void initializeVectorsForForward(); 
  static std::vector<Node3D> interpolateDirect(const Node3D& start, const Node3D& end, float interval);
  static int widthForMap;
  static int heightForMap;
 private:
  float x;
  float y;
  float t;
  float g;
  float h;
  int idx;
  bool o;
  bool c;
  int prim;
  const Node3D* pred;
};
}
#endif 
