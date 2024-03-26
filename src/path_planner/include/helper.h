
#ifndef HELPER
#define HELPER

#include <cmath>
#include <algorithm>
#include <node2d.h>
#include "constants.h"
#include <opencv2/opencv.hpp>
namespace HybridAStar {

namespace Helper {


static inline float normalizeHeading(float t) {
  if ((int)t <= 0 || (int)t >= 360) {
    if (t < -0.1) {
      t += 360.f;
    } else if ((int)t >= 360) {
      t -= 360.f;
    } else {
      t =  0;
    }
  }

  return t;
}


static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}


static inline float toDeg(float t) {
  return normalizeHeadingRad(t) * 180.f / M_PI ;
}


static inline float toRad(float t) {
  return normalizeHeadingRad(t / 180.f * M_PI);
}


static inline float clamp(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

static inline float crossProduct(const Node2D* a,const Node2D* b,const Node2D* c) {
    float y1 = b->getFloatY() - a->getFloatY();
    float y2 = c->getFloatY() - a->getFloatY();
    float x1 = b->getFloatX() - a->getFloatX();
    float x2 = c->getFloatX() - a->getFloatX();
    return x1 * y2 - x2 * y1;
}


static inline bool isIntersect(const Node2D* a, const Node2D* b,const Node2D* c,const Node2D* d) {
    float d1 = crossProduct(a, b, c);
    float d2 = crossProduct(a, b, d);
    float d3 = crossProduct(c, d, a);
    float d4 = crossProduct(c, d, b);

    
    if (d1 * d2 < 0 && d3 * d4 < 0) return true;
    return false;
}
static float PixelDistance(const cv::Point2f& pre_p, const cv::Point2f& cur_p) {
  return std::hypotf(pre_p.x - cur_p.x, pre_p.y - cur_p.y);
}

}
}

#endif 

