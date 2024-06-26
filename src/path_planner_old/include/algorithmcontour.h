#ifndef ALGORITHM_CONTOUR_H
#define ALGORITHM_CONTOUR_H
#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"
#include "constants.h"
#include <cmath>
#include <directionVector.h>
#include <boost/filesystem.hpp>
namespace HybridAStar {
  class keyInfoForThrouthNarrowPair{
   public:
   directionVector wireUnitVector;
   Node2D* centerPoint;
   directionVector centerVerticalUnitVector;
   Node2D* firstBoundPoint;
   Node2D firstBoundRealPoint;
   Node2D* secondBoundPoint;
   Node2D secondBoundRealPoint;
   Node3D centerVerticalPoint3D;
   std::vector<Node3D> containingWaypointsFirstBPForward;
   std::vector<Node3D> containingWaypointsFirstBPBackward; 
   std::vector<Node3D> containingWaypointsSecondBPForward;
   std::vector<Node3D> containingWaypointsSecondBPBackward; 
   bool whetherCloseReverseToGoal = false;
   Node3D getSecondStageMiddleVerticalPoint();
  };
  class finalPassSpaceInOutSet{
  public:
    std::vector<Node3D> inSet;
    std::vector<Node3D> outSet;
    finalPassSpaceInOutSet(std::vector<Node3D> inSet,std::vector<Node3D> outSet){
      this->inSet = inSet;
      this->outSet = outSet;
    }
    finalPassSpaceInOutSet(){
      this->inSet = std::vector<Node3D>();
      this->outSet = std::vector<Node3D>();
    }
  };
  class AlgorithmContour {
  public:
    AlgorithmContour() {}
    const static bool WhetherDebug = false;
    const static bool whetherDeepDebug = false; 
    const static bool whetherDeepDebug2 = false;
    const static int visualizeMultiplier = 2; 
    cv::Mat gridMap;
    std::vector<std::vector<Node2D*>> contoursFromGrid;
    std::vector<std::pair<Node2D*, Node2D*>> narrowPairs;
    std::vector<std::pair<Node2D*, Node2D*>> throughNarrowPairs;
    std::vector<std::vector<Node2D>> throughNarrowPairsWaypoints; 
    std::vector<int> aroundWaypointsIndexOfThroughNarrowPairs; 
    std::vector<keyInfoForThrouthNarrowPair*> keyInfoForThrouthNarrowPairs;
    std::vector<finalPassSpaceInOutSet> finalPassSpaceInOutSets;
    std::vector<std::vector<cv::Point2f>> refineContours;
    std::vector<std::vector<Node2D*>> findContour(nav_msgs::OccupancyGrid::Ptr grid);
    int TPCAP_index = -1;
    void findNarrowContourPair();
    void findThroughNarrowContourPair(const std::vector<Node2D> & path);
    bool determineWhetherThrough2DPath(const std::vector<Node2D> & path, std::pair<Node2D*, Node2D*> narrowPair,std::vector<Node2D> & containingWaypointsTorecord,int & aroundWaypointsIndex);
    void sortThroughNarrowPairsWaypoints();
    void findKeyInformationForThrouthNarrowPairs(); 
    static bool samplePathAndjudgeAcuteAngel(std::vector<Node2D> & path,directionVector midperpendicular);
    static void visualizeNarrowPairs(std::vector<std::pair<Node2D*,Node2D*>> narrowPairs, const cv::Mat & gridMap);
    static void visualizePathAndItNarrowPair(std::vector<Node2D> & path,std::pair<Node2D*,Node2D*> narrowPair,const cv::Mat & gridMap);
    static void visualizekeyInfoForThrouthNarrowPair(std::pair<Node2D*,Node2D*> narrowPair,keyInfoForThrouthNarrowPair* keyInfo,const cv::Mat & gridMap);
    static void visualizeInsetForThroughNarrowPairs(std::vector<finalPassSpaceInOutSet> finalPassSpaceInOutSets,const cv::Mat & gridMap);
    std::vector<Node3D> findNarrowPassSpace(CollisionDetection& configurationSpace,const directionVector& radiusVectorToYuanxin,const directionVector& tangentVector,Node2D* startPoint, int whetherReverse, int whetherCloseReverseGoal);
    void findNarrowPassSpaceForAllPairs(CollisionDetection& configurationSpace,const Node3D & goal);
    static void visualizePassSpaceBoundaryForThroughNarrowPair(keyInfoForThrouthNarrowPair* keyInfo,const cv::Mat & gridMap);
    void findNarrowPassSpaceInputSetOfNode3DForAllPairs(CollisionDetection& configurationSpace);
    finalPassSpaceInOutSet findNarrowPassSpaceInputSetOfNode3D(CollisionDetection& configurationSpace,keyInfoForThrouthNarrowPair* inputPair);
    std::vector<Node3D> findInputSetOfNode3DByTwoVectorAndMiddleVerticalLine(const Node3D & firstPoint,const Node3D & secondPoint,const Node2D & middlePoint,const directionVector middleVerticalLine,int whetherCloseReverseGoal);
    void AdjecentDistanceFilter(std::vector<std::vector<cv::Point2f>>& contoursInOut);
    bool two3DPointsWhetherCloseAndReverseDirection(CollisionDetection& configurationSpace,
    Node2D * middlePoint,directionVector& centerVerticalUnitVector,const Node3D & goal);
    void savePicturePathAndContour(std::vector<Node2D> path2D);
    void savePicturePairAndKeyInformation();
    void savePictureNarrowSpaceBoundary();
    void savePictureNarrowSpaceInputSet();
    void saveMapCsv(Node3D start,Node3D goal);
    inline void RemoveWallConnection(const std::vector<cv::Point2f>& contour,
                                      const cv::Point2f& add_p,
                                      std::size_t& refined_idx)
    {
        if (refined_idx < 2) return;
        if (!IsPrevWallVertex(contour[refined_idx-2], contour[refined_idx-1], add_p)) {
            return;
        } else {
            -- refined_idx;
            RemoveWallConnection(contour, add_p, refined_idx);
        }
    }
    inline bool IsPrevWallVertex(const cv::Point2f& first_p,
                                  const cv::Point2f& mid_p,
                                  const cv::Point2f& add_p)
      {
          cv::Point2f diff_p1 = first_p - mid_p;
          cv::Point2f diff_p2 = add_p - mid_p;
          diff_p1 /= std::hypotf(diff_p1.x, diff_p1.y);
          diff_p2 /= std::hypotf(diff_p2.x, diff_p2.y);
          if (abs(diff_p1.dot(diff_p2)) > Constants::ALIGN_ANGLE_COS) return true;
          return false;
      }
    };
}
#endif 
