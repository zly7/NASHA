#ifndef NODE2D_H
#define NODE2D_H

#include <cmath>
#include <cassert>
#include "constants.h"
#include <iostream>
namespace HybridAStar {


class Node2D {
 public:
  
  Node2D(): Node2D(0, 0, 0, 0, nullptr) {}
  
  Node2D(int x, int y, float g, float h, Node2D* pred) {
    this->x = x;
    this->y = y;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->d = false;
    this->idx = this->y * Node2D::widthForMap + this->x;
    this->float_x = static_cast<float>(x);
    this->float_y = static_cast<float>(y);
    this->wide = false;
    this->boundary = false;
    this->radius = 0;
  }

  Node2D(float x, float y) { 
    this->x = int(x);
    this->y = int(y);
    this->g = 0;
    this->h = 0;
    this->pred = nullptr;
    this->o = false;
    this->c = false;
    this->d = false;
    this->idx = this->y * Node2D::widthForMap + this->x;
    this->radius = 0;
    this->float_x = x;
    this->float_y = y;
    this->wide = false;
    this->boundary = false;
  }
  
  
  int getX() const { return x; }
  
  int getY() const { return y; }
  
  float getG() const { return g; }
  
  float getH() const { return h; }
  
  float getC() const { return g + h; }
  
  int getIdx() const { return idx; }
  
  bool  isOpen() const { return o; }
  
  bool  isClosed() const { return c; }

  bool  isDiscovered() const { return d; }
  
  Node2D* getPred() const { return pred; }

  float getRadius() const { return radius;}
  int getIntX() const { return x;}
  int getIntY() const { return y;}
  float getFloatX() const { return float_x;}
  float getFloatY() const { return float_y;}
  bool getWide() const { return wide;}
  bool getBoundary() const { return boundary;}

  
  
  void setX(const int& x) { this->x = x; }
  
  void setY(const int& y) { this->y = y; }
  
  void setG(const float& g) { this->g = g; }
  
  void setH(const float& h) { this->h = h; }
  
  
  
  void open() { o = true; c = false; }
  
  void close() { c = true; o = false; }
  
  void reset() { c = false; o = false; }

  void discover() { d = true; }
  
  void setPred(Node2D* pred) { this->pred = pred; }

  void setRadius(const float& radius) { this->radius = radius; }
  void setFloatx(const float& ix) { this->float_x = ix; }
  void setFloaty(const float& iy) { this->float_y = iy; }
  void setWide(const bool& wide) { this->wide = wide; }
  void setBoundary(const bool&boundary) { this->boundary = boundary; }

  float distanceTo(const Node2D* other) const {
    return sqrt(pow(float_x - other->float_x, 2) + pow(float_y - other->float_y, 2));
  }
  
  
  void updateG() { g += movementCost(*pred); d = true; }
  
  void updateH(const Node2D& goal) { h = movementCost(goal); }
  
  float movementCost(const Node2D& pred) const { return sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y)); }

  
  
  bool operator == (const Node2D& rhs) const;

  
  
  bool isOnGrid(const int width, const int height) const;

  
  
  Node2D* createSuccessor(const int i);

  Node2D* getSuccessor(const int i,Node2D* nodeArray,const int width,const int height);

  
  
  static const int dir;
  
  static const int dx[];
  
  static const int dy[];

  bool whether_store_float;

  
  static Node2D* middle(const Node2D* a, const Node2D* b) {
      return new Node2D((a->float_x + b->float_x) / 2, (a->float_y + b->float_y) / 2);
  }
  static int widthForMap;
 private:
  
  int x;
  
  int y;
  
  float g;
  
  float h;
  
  int idx;
  
  bool o;
  
  bool c;
  
  bool d;
  
  Node2D* pred;

  float radius;

  float float_x;
  float float_y;

  bool wide;
  bool boundary;

};
}
#endif 
