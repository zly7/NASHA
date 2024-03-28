#ifndef CONSTANTS
#define CONSTANTS
#include <cmath>
#include <string>
#include <vector>
namespace HybridAStar {
namespace Constants {
static const std::string algorithm = "hybrid_astar";
static const bool coutDEBUG = true;
static const bool manual = true;
static const bool visualization = true && manual;
static const bool visualizationStartAndGoal = true && manual;
static const bool visualization2D = true && manual;
static const bool reverse = true;
static const bool dubinsShot = true;
static const bool randomDubinsShot = true && dubinsShot;
static const float useDubinsShotMinDeltaAngel = (120.0/180.0) * M_PI ;
static const bool dubins = false;
static const bool dubinsLookup = false && dubins;
static const bool twoD = true;
static const bool useDubinReedSheepHeuristic = true;
static const float heuristicDecayCoefficient = algorithm=="contour_hybrid_astar"? 0.95:0.98;
static const int iterations = 5000000;
static const int iterationsToPrint = 10000;
static const double bloating = 0;
static const double each_meter_to_how_many_pixel = 10.0;
static const double width = 1.942 * each_meter_to_how_many_pixel + 2 * bloating;
static const bool useRearAsCenter = true;
static const double frontHangLength = 0.960 * each_meter_to_how_many_pixel;
static const double wheelBase = 2.8 * each_meter_to_how_many_pixel;
static const double rearHangLength = 0.929 * each_meter_to_how_many_pixel;
static const double length = frontHangLength + wheelBase + rearHangLength + 2 * bloating;
static const float r = 3.0059 * each_meter_to_how_many_pixel;
static const int headings = 72;
static const float deltaHeadingDeg = 360 / (float)headings;
static const float deltaHeadingRad = 2 * M_PI / (float)headings;
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
static const float cellSize = 1;
static const float tieBreaker = 0.01;
static const float factor2D = sqrt(5) / sqrt(2) + 1;
static const float penaltyTurning = 1.1;
static const float penaltyReversing = 1.05;
static const float penaltyCOD = 5.0;
static const float dubinsShotMAXDistance =  10 * length;
static const float dubinsShotMINDistance = 0.5 * length;
static const float arcShotDistance = 2 * length;
static const float deltaL1 = 1;
static const float deltaL2 = 0.1;
static const float deltaLTest = 0.3;
static const int dubinsWidth = 15;
static const int dubinsArea = dubinsWidth * dubinsWidth;
static const int bbSize = std::ceil((sqrt(width * width + 2*2*(wheelBase+frontHangLength)*(wheelBase+frontHangLength)) + 4));
static const int positionResolution = 4;
static const int positions = positionResolution * positionResolution;
struct relPos {
  int x;
  int y;
};
struct config {
  int length;
  std::vector<relPos> pos;
};
static const float minRoadWidth = 2;
struct color {
  float red;
  float green;
  float blue;
};
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
static constexpr color cyan = {139.f / 255.f, 240.f / 255.f, 103.f / 255.f};
static constexpr color brown = {139.f / 255.f, 69.f / 255.f, 19.f / 255.f};
static constexpr color black = {0.f / 255.f, 0.f / 255.f, 0.f / 255.f};
static constexpr color red = {255.f / 255.f, 0.f / 255.f, 0.f / 255.f};
static constexpr color blue = {0.f / 255.f, 191.f / 255.f, 255.f / 255.f};
static float maxAngleRag = M_PI * 60 /180 ;
static float minRadius = r;
static float  maxRadus = std::min(6 * r, 30* (float)each_meter_to_how_many_pixel);
static float deltaRadius = 0.1 * r;
static float theMindistanceDetermineWhetherTheSameContourPoint = 0.6 * each_meter_to_how_many_pixel;
static const float minContourPairDistance = width * 1;
static const float maxContourPairDistance = width * 1.65;
static const float maxNarrowSpaceArcLength = length * 1.25; 
static const int howManyNode2DDeterminesWhetherThroughNarrowContourPair = 3;
static const int howManyLevelInputPick  = 5; 
static const float interpolateGapForInputNarrowSpace = 2;
static float offsetPercentForHalfVehicleWidth = 1.1;
static const float arcLengthForAstarSuccessor = length / 24;
static const float dubinsStepSize = arcLengthForAstarSuccessor;
static const float radiusForAstarSuccessor = r * 1.05;
static const int toleranceForCollisionCheck = 2;
static const float DIST_LIMIT = 1.5;
static const float ALIGN_ANGLE_COS = cos(15.0/180.0 * M_PI);
static const float theDistanceDerterminReverseMiddleDirection= length * 1.5;
static const bool whetherFuzzyGoal = false && each_meter_to_how_many_pixel >= 6;
static const bool useArcShot = false;
static const float tolerance = 0.3 * each_meter_to_how_many_pixel;
static const float fuzzyLength = length * 0.25;
static const bool useAutoTest = true;
static const float findNarrowSpaceMoveDistance = deltaHeadingRad * minRadius;
static const float drwaOffset = 0.05 * each_meter_to_how_many_pixel;
static const bool whetherSplitSearch = false;
static const bool useRandomGeneratingSuccessor = false;
static const bool saveMapCsv =  true;
static const float reverseBackDistance = wheelBase * 0.6;
}
}
#endif 
