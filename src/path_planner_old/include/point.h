#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_
#define INTPOINT IntPoint
namespace HybridAStar {
class IntPoint {
 public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  int x, y;
};
}
#endif