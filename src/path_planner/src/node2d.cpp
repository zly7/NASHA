#include "node2d.h"

using namespace HybridAStar;


const int Node2D::dir = 8;
int Node2D::widthForMap = 0;

const int Node2D::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int Node2D::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };




bool Node2D::isOnGrid(const int width, const int height) const {
  return  x >= 0 && x < width && y >= 0 && y < height;
}




Node2D* Node2D::createSuccessor(const int i) {
  int xSucc = x + Node2D::dx[i];
  int ySucc = y + Node2D::dy[i];
  return new Node2D(xSucc, ySucc, g, 0, this);
}

Node2D* Node2D::getSuccessor(const int i,Node2D* nodeArray,const int width,const int height) {
  int xSucc = x + Node2D::dx[i];
  int ySucc = y + Node2D::dy[i];
  if (xSucc >= 0 && xSucc < width && ySucc >= 0 && ySucc < height) {
    return &nodeArray[xSucc + ySucc * width];
  }
  return nullptr;
}




bool Node2D::operator == (const Node2D& rhs) const {
  return x == rhs.x && y == rhs.y;
}
