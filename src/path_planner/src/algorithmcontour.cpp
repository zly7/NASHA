#include "algorithmcontour.h"
#include <ros/package.h>
#include <boost/heap/binomial_heap.hpp>
#include <CGAL/Cartesian.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <unordered_set>
#include <yaml-cpp/yaml.h>
#include <regex>
using namespace HybridAStar;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Line_2 Line_2;


Node3D keyInfoForThrouthNarrowPair::getSecondStageMiddleVerticalPoint(){
  float tempT;
  if(this->whetherCloseReverseToGoal){
    tempT = Helper::normalizeHeadingRad(centerVerticalPoint3D.getT() + M_PI);
    return Node3D(centerVerticalPoint3D.getX(),centerVerticalPoint3D.getY(),tempT,0,0,nullptr);
  }else{
    tempT = centerVerticalPoint3D.getT();
    float x = centerVerticalPoint3D.getX() - Constants::reverseBackDistance * cos(tempT);
    float y = centerVerticalPoint3D.getY() - Constants::reverseBackDistance * sin(tempT);
    return Node3D(x,y,tempT,0,0,nullptr);
  }
}
std::vector<std::vector<Node2D*>> AlgorithmContour::findContour(nav_msgs::OccupancyGrid::Ptr grid){
  
  cv::Mat img(grid->info.height, grid->info.width, CV_8UC1);
  for (uint i = 0; i < grid->data.size(); i++) {
    img.data[i] = grid->data[i] >=1 ? 255 : 0; 
  }
  if(WhetherDebug){
    cv::imshow("img", img);
    cv::waitKey(0);
  }
  this->gridMap = img;
  
  std::vector<std::vector<cv::Point2i>> rawContours;
  std::vector<std::vector<cv::Point2f>> refineContours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(img, rawContours, hierarchy, cv::RetrievalModes::RETR_TREE, 
                     cv::ContourApproximationModes::CHAIN_APPROX_TC89_L1);
  refineContours.resize(rawContours.size());
  for (std::size_t i=0; i<rawContours.size(); i++) {
        
        cv::approxPolyDP(rawContours[i], refineContours[i], Constants::DIST_LIMIT, true);
  }
  AdjecentDistanceFilter(refineContours);
  if(WhetherDebug){
      cv::Mat contourImg = cv::Mat::zeros(img.size(), CV_8UC3);
      std::vector<std::vector<cv::Point>> rContoursForDraw;
      for (const auto& contour : refineContours) {
          std::vector<cv::Point> intContour;
          for (const auto& point : contour) {
              intContour.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
          }
          rContoursForDraw.push_back(intContour);
      }
      for (uint i = 0; i < rContoursForDraw.size(); i++) {
        cv::Scalar color = cv::Scalar(0, 255, 0); 
        cv::Scalar pointColor = cv::Scalar(0, 0, 255);
        cv::drawContours(contourImg, rContoursForDraw, static_cast<int>(i), color, 2, cv::LINE_8, hierarchy, 0);
        
        for (uint j = 0; j < rContoursForDraw[i].size(); j++) {
            cv::circle(contourImg, rContoursForDraw[i][j], 2, pointColor, -1); 
        }
      }
      cv::imshow("Contours", contourImg);
      cv::waitKey(0);
  }
  
  std::vector<std::vector<Node2D*>> result;
  for (auto& c : refineContours) {
    std::vector<Node2D*> contour;
    for (auto& p : c) {
      Node2D* node = new Node2D(((float)p.x)+0.5, ((float)p.y)+0.5);
      contour.push_back(node);
    }
    result.push_back(contour);
  }
  this->contoursFromGrid = result;
  if(Constants::saveMapCsv){
    this->refineContours = refineContours;
  }
  
  return result;
}
void AlgorithmContour::saveMapCsv(Node3D start,Node3D goal){
  
  std::string packagePath = "/home/zly/plannerAll/catkin_path_planner";
  std::string yamlPath = packagePath + "/src/path_planner/maps/map.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);
  std::string unique_number = "-1";
  if (config["image"]) {
    std::string map_image = config["image"].as<std::string>();
    size_t tpcap_pos = map_image.find("TPCAP");
    if (tpcap_pos != std::string::npos) {
      std::regex number_regex("(\\d+)");
      std::smatch match;
      if (std::regex_search(map_image, match, number_regex)) {
          
          unique_number = match[0].str();
      }
    }
  }
  std::string csvPath = packagePath + "/mapCsv/Case"+unique_number+".csv";
  this->TPCAP_index = std::stoi(unique_number);
  float mul = 0.1;
  std::string csvMulPath = packagePath + "/mapCsv/Case"+unique_number+"Mul"+std::to_string(static_cast<int>(1/mul))+".csv";
  auto saveScaledMap = [&](float scale, const std::string& scaledCsvPath) {
      std::ofstream scaledFile(scaledCsvPath);
      if (!scaledFile.is_open()) {
          std::cerr << "Error opening file: " << scaledCsvPath << std::endl;
          return;
      }
      scaledFile << start.getX() * scale << "," << start.getY() * scale << "," << start.getT() 
                  << "," << goal.getX() * scale << "," << goal.getY() * scale << "," << goal.getT();
      scaledFile << "," << refineContours.size();
      for (const auto& contour : refineContours) {
        scaledFile << "," << contour.size();
      }
      for (const auto& contour : refineContours) {
          for (const auto& point : contour) {
              scaledFile << "," << point.x * scale << "," << point.y * scale;
          }
      }
      scaledFile.close();
  };
  saveScaledMap(1.0, csvMulPath);
  saveScaledMap(mul, csvPath);
}

void AlgorithmContour::AdjecentDistanceFilter(std::vector<std::vector<cv::Point2f>>& contoursInOut) {
    
    std::unordered_set<int> remove_idxs;
    for (std::size_t i=0; i<contoursInOut.size(); i++) { 
        const auto c = contoursInOut[i];
        const std::size_t c_size = c.size();
        std::size_t refined_idx = 0;
        for (std::size_t j=0; j<c_size; j++) {
            cv::Point2f p = c[j]; 
            if (refined_idx < 1 || Helper::PixelDistance(contoursInOut[i][refined_idx-1], p) > Constants::DIST_LIMIT) {
                
                RemoveWallConnection(contoursInOut[i], p, refined_idx);
                contoursInOut[i][refined_idx] = p;
                refined_idx ++;
            }
        }
        
        RemoveWallConnection(contoursInOut[i], contoursInOut[i][0], refined_idx);
        contoursInOut[i].resize(refined_idx);
        if (refined_idx > 1 && Helper::PixelDistance(contoursInOut[i].front(), contoursInOut[i].back()) < Constants::DIST_LIMIT) {
            contoursInOut[i].pop_back();
        }
        if (contoursInOut[i].size() < 3) remove_idxs.insert(i);
    }
    if (!remove_idxs.empty()) { 
        std::vector<std::vector<cv::Point2f>> temp_contours = contoursInOut;
        contoursInOut.clear();
        for (uint i=0; i<temp_contours.size(); i++) {
            if (remove_idxs.find(i) != remove_idxs.end()) continue;
            contoursInOut.push_back(temp_contours[i]);
        }
    }
}

void AlgorithmContour::findNarrowContourPair(){
    float minDistance = Constants::minContourPairDistance;
    float maxDistance = Constants::maxContourPairDistance;
    

    std::vector<Node2D*> allNodes;
    for (const auto& contour : this->contoursFromGrid) {
        allNodes.insert(allNodes.end(), contour.begin(), contour.end());
    }
    std::set<size_t> nodesToRemove;
    std::vector<Node2D*> middleNodes;

    
    for (size_t i = 0; i < allNodes.size(); ++i) {
        for (size_t j = i + 1; j < allNodes.size(); ++j) {
            float distance = allNodes[i]->distanceTo(allNodes[j]);
            if (distance < Constants::theMindistanceDetermineWhetherTheSameContourPoint) {
                middleNodes.push_back(Node2D::middle(allNodes[i], allNodes[j]));
                nodesToRemove.insert(i);
                nodesToRemove.insert(j);
            }
        }
    }

    
    for (auto it = nodesToRemove.rbegin(); it != nodesToRemove.rend(); ++it) {
        allNodes.erase(allNodes.begin() + *it);
    }
      
    allNodes.insert(allNodes.end(), middleNodes.begin(), middleNodes.end());
    for (size_t i = 0; i < allNodes.size(); ++i) {
        for (size_t j = i + 1; j < allNodes.size(); ++j) {
            float distance = allNodes[i]->distanceTo(allNodes[j]);
            if (distance > minDistance && distance < maxDistance) {
                this->narrowPairs.emplace_back(allNodes[i], allNodes[j]);
            }
        }
    }
   
    return;
}

void AlgorithmContour::findThroughNarrowContourPair(const std::vector<Node2D> & path){

    for (const auto& narrowPair : this->narrowPairs) {
        std::vector<Node2D> containingWaypointsTorecord;
        int aroundWaypointsIndex = 0;
        
        
        
        
        
        
        if (this->determineWhetherThrough2DPath(path, narrowPair,containingWaypointsTorecord,aroundWaypointsIndex)) {
            Node2D * nodeRepeat1 = narrowPair.first;
            Node2D * nodeRepeat2 = narrowPair.second;
            bool whetherSkip = false;
            for( uint i = 0;i < this->throughNarrowPairs.size();i++){
              std::pair<Node2D*, Node2D*> nodePair = this->throughNarrowPairs[i];
              if(nodeRepeat1 == nodePair.first || nodeRepeat1 == nodePair.second || nodeRepeat2 == nodePair.first || nodeRepeat2 == nodePair.second){
                if(nodeRepeat1->distanceTo(nodeRepeat2) > nodePair.first->distanceTo(nodePair.second)){
                  whetherSkip = true;
                  break;
                }else{
                  this->throughNarrowPairs.erase(this->throughNarrowPairs.begin()+i);
                  this->aroundWaypointsIndexOfThroughNarrowPairs.erase(this->aroundWaypointsIndexOfThroughNarrowPairs.begin()+i);
                  this->throughNarrowPairsWaypoints.erase(this->throughNarrowPairsWaypoints.begin()+i);
                  break;
                }

              }
            }
            if(whetherSkip){
              continue;
            }
            this->throughNarrowPairs.push_back(narrowPair);
            this->aroundWaypointsIndexOfThroughNarrowPairs.push_back(aroundWaypointsIndex);
            
            this->throughNarrowPairsWaypoints.push_back(containingWaypointsTorecord);
        }
    }
    return;
};


bool AlgorithmContour::determineWhetherThrough2DPath(const std::vector<Node2D>& path, 
          std::pair<Node2D*, Node2D*> narrowPair,std::vector<Node2D> & containingWaypointsTorecord,int & aroundWaypointsIndex){
    int minContinue = Constants::howManyNode2DDeterminesWhetherThroughNarrowContourPair;
    int continueCount = 0;
    Node2D* pairFirst = narrowPair.first;
    Node2D* pairSecond = narrowPair.second;
    
    float maxDistance = pairFirst->distanceTo(pairSecond);
    int index = 0;
    int allIndex = 0;
    bool isFlag = false;
    for (const auto& node : path) {
        if (node.distanceTo(narrowPair.first) < maxDistance && node.distanceTo(narrowPair.second) < maxDistance) {
            continueCount++;
            containingWaypointsTorecord.push_back(node);
            allIndex += index;
        }
        else {
            continueCount = 0;
            if(isFlag==true){
              break;
            }else{
              allIndex = 0;
              containingWaypointsTorecord.clear();
            }
        }
        if (continueCount >= minContinue) {
            isFlag = true;
        }
        index++;
    }
    if(isFlag==true){
      aroundWaypointsIndex = allIndex / containingWaypointsTorecord.size();
    }
    
    if(isFlag==true && !Helper::isIntersect(pairFirst, pairSecond, &containingWaypointsTorecord[0], &containingWaypointsTorecord[containingWaypointsTorecord.size() - 1])){
        isFlag = false;
    }
    
    
    
    
    return isFlag;
}

void AlgorithmContour::sortThroughNarrowPairsWaypoints(){
  
  if (throughNarrowPairsWaypoints.size() != aroundWaypointsIndexOfThroughNarrowPairs.size()) {
      std::cout << "Error: throughNarrowPairsWaypoints.size() != aroundWaypointsIndexOfThroughNarrowPairs.size()" << std::endl;
      return;
  }

  
  std::vector<size_t> indices(throughNarrowPairsWaypoints.size());
  for (size_t i = 0; i < indices.size(); ++i) {
      indices[i] = i;
  }

  
  std::sort(indices.begin(), indices.end(), [this](size_t a, size_t b) {
      return aroundWaypointsIndexOfThroughNarrowPairs[a] < aroundWaypointsIndexOfThroughNarrowPairs[b];
  });

  
  std::vector<std::vector<Node2D>> sortedWaypoints(throughNarrowPairsWaypoints.size());
  std::vector<std::pair<Node2D*, Node2D*>> sortedThroughNarrowPairs(throughNarrowPairsWaypoints.size());
  for (size_t i = 0; i < indices.size(); ++i) {
      sortedWaypoints[i] = throughNarrowPairsWaypoints[indices[i]];
      sortedThroughNarrowPairs[i] = throughNarrowPairs[indices[i]];
  }

  
  throughNarrowPairsWaypoints = sortedWaypoints;
  throughNarrowPairs = sortedThroughNarrowPairs;
}

void AlgorithmContour::findKeyInformationForThrouthNarrowPairs(){
  int index = 0;
  float offsetForHalfVehicleWidth = ((float) (Constants::width+1)) / 2.0 * Constants::offsetPercentForHalfVehicleWidth; 
  for (const auto& pair : this->throughNarrowPairs) {
      Node2D* firstPoint = pair.first;
      Node2D* secondPoint = pair.second;

      
      keyInfoForThrouthNarrowPair* keyInfo = new keyInfoForThrouthNarrowPair();
      this->keyInfoForThrouthNarrowPairs.push_back(keyInfo);
      directionVector unitWireVector{};
      unitWireVector.x = secondPoint->getFloatX() - firstPoint->getFloatX();
      unitWireVector.y = secondPoint->getFloatY() - firstPoint->getFloatY();
      unitWireVector.normalize();
      keyInfo->wireUnitVector = unitWireVector;
      Node2D* centerPoint = new Node2D((firstPoint->getFloatX() + secondPoint->getFloatX()) / 2,
                                        (firstPoint->getFloatY() + secondPoint->getFloatY()) / 2);
      keyInfo->centerPoint = centerPoint;
      
      directionVector centerVerticalUnitVector{unitWireVector.y,-unitWireVector.x};
      if(!samplePathAndjudgeAcuteAngel(this->throughNarrowPairsWaypoints[index],centerVerticalUnitVector)){
        centerVerticalUnitVector.x = -1*centerVerticalUnitVector.x;
        centerVerticalUnitVector.y = -1*centerVerticalUnitVector.y;
      }
      keyInfo->centerVerticalUnitVector = centerVerticalUnitVector;
      float tempT = Helper::normalizeHeadingRad(atan2f(centerVerticalUnitVector.y,centerVerticalUnitVector.x));
      Node3D centerVerticalPoint(centerPoint->getFloatX() , 
                                              centerPoint->getFloatY() , 
                                              tempT, 0, 0, nullptr);
      keyInfo->centerVerticalPoint3D = centerVerticalPoint;
      Node2D* firstBoundPoint = new Node2D(firstPoint->getFloatX() + offsetForHalfVehicleWidth * unitWireVector.x, 
                                          firstPoint->getFloatY() + offsetForHalfVehicleWidth * unitWireVector.y);
      Node2D* secondBoundPoint = new Node2D(secondPoint->getFloatX() - offsetForHalfVehicleWidth * unitWireVector.x, 
                                            secondPoint->getFloatY() - offsetForHalfVehicleWidth * unitWireVector.y);

      keyInfo->firstBoundPoint = firstBoundPoint;
      keyInfo->secondBoundPoint = secondBoundPoint;
      index++;
  }
};

bool AlgorithmContour::samplePathAndjudgeAcuteAngel(std::vector<Node2D> & path,directionVector midperpendicular){
  float allDot = 0;
  for(uint i =0;i<path.size()-3;i++){
    Node2D* nodeStart = &path[i];
    float x0 = nodeStart->getFloatX();
    float y0 = nodeStart->getFloatY();
    Node2D* nodeEnd = &path[i+3];
    float x1 = nodeEnd->getFloatX();
    float y1 = nodeEnd->getFloatY();
    float x = x1 - x0;
    float y = y1 - y0;
    directionVector vector{x,y};
    vector.normalize();
    float dot = vector.x * midperpendicular.x + vector.y * midperpendicular.y;
    allDot += dot;
  }
  if(allDot > 0){
    return true;
  }else{
    return false;
  }
}

bool AlgorithmContour::two3DPointsWhetherCloseAndReverseDirection(CollisionDetection& configurationSpace,
    Node2D * middlePoint,directionVector& centerVerticalUnitVector, const Node3D & goal){
  
    float x1 = middlePoint->getFloatX();
    float y1 = middlePoint->getFloatY();
    float x2 = goal.getX();
    float y2 = goal.getY();
    float distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    if (distance >= Constants::theDistanceDerterminReverseMiddleDirection) {
        return false; 
    }
    int steps = int(sqrt((pow(x2 - x1, 2) + pow(y2 - y1, 2))));
    for (int i = 0; i <= steps; ++i) {
        float ratio = static_cast<float>(i) / steps;
        float x = x1 + ratio * (x2 - x1);
        float y = y1 + ratio * (y2 - y1);
        Node2D* nodeInterpolation = new Node2D(x, y);
        if (!configurationSpace.isTraversablePreciseFor2DWithTolerance(nodeInterpolation)) {
            std::cout << "在判断是否应该反向目标点的函数里面直线的插值不能通过，这种情况应该极少出现！" << std::endl;
            delete nodeInterpolation;
            return false; 
        }
        delete nodeInterpolation;
    }

    float t =goal.getT();
    directionVector goalDirection = {cos(t),sin(t)};

    
    float dotProduct = goalDirection.x * centerVerticalUnitVector.x + goalDirection.y * centerVerticalUnitVector.y;
    
    if (dotProduct <= -0.5) { 
        return true; 
    }
    return false; 
}


void AlgorithmContour::visualizeNarrowPairs(std::vector<std::pair<Node2D*,Node2D*>> narrowPairs, const cv::Mat & gridMap){
    int pairsCount = narrowPairs.size();
    int howManyCols = std::min((int)ceil(sqrt(pairsCount)), 3);
    int rows = ceil(pairsCount / ((float)howManyCols));
    rows = std::min(rows, 3);

    int singleWidth = gridMap.cols;
    int singleHeight = gridMap.rows;
    int pairsPerImage = rows * howManyCols;
    int totalImages = ceil(pairsCount / (float)pairsPerImage);

    for (int img = 0; img < totalImages; ++img) {
        cv::Mat bigImage(rows * singleHeight, howManyCols * singleWidth, CV_8UC3);

        for (int i = img * pairsPerImage; i < std::min((img + 1) * pairsPerImage, pairsCount); ++i) {
            cv::Mat mapCopy;
            cv::cvtColor(gridMap, mapCopy, cv::COLOR_GRAY2BGR);
            
            cv::circle(mapCopy, cv::Point(narrowPairs[i].first->getFloatX(), narrowPairs[i].first->getFloatY()), Constants::each_meter_to_how_many_pixel*0.5, cv::Scalar(0, 0, 255), -1);
            cv::circle(mapCopy, cv::Point(narrowPairs[i].second->getFloatX(), narrowPairs[i].second->getFloatY()), Constants::each_meter_to_how_many_pixel*0.5, cv::Scalar(255, 0, 0), -1);

            int row = (i % pairsPerImage) / howManyCols;
            int col = (i % pairsPerImage) % howManyCols;
            mapCopy.copyTo(bigImage(cv::Rect(col * singleWidth, row * singleHeight, singleWidth, singleHeight)));
        }

        cv::Size newSize(700, 700 * gridMap.rows / gridMap.cols);
        cv::Mat resizedImage;
        cv::resize(bigImage, resizedImage, newSize);
        cv::imshow("Resized Narrow Pairs Visualization Image " + std::to_string(img), resizedImage);
        cv::waitKey(0);
    }
}


void AlgorithmContour::visualizePathAndItNarrowPair(std::vector<Node2D> & path, std::pair<Node2D*, Node2D*> narrowPair, const cv::Mat & gridMap){
    float mul = AlgorithmContour::visualizeMultiplier;  
    cv::Mat mapCopy;
    cv::resize(gridMap, mapCopy, cv::Size(), mul, mul, cv::INTER_LINEAR);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
    cv::circle(mapCopy, cv::Point(narrowPair.first->getFloatX() * mul, narrowPair.first->getFloatY() * mul), 3, cv::Scalar(0, 0, 255), -1);
    cv::circle(mapCopy, cv::Point(narrowPair.second->getFloatX() * mul, narrowPair.second->getFloatY() * mul), 3, cv::Scalar(255, 0, 0), -1);
    for(uint i = 0; i < path.size(); i++){
        cv::circle(mapCopy, cv::Point(path[i].getFloatX() * mul, path[i].getFloatY() * mul), 2, cv::Scalar(0, 255, 0), -1);
    }
    cv::Size newSize(500 * gridMap.cols / gridMap.rows , 500 );
    cv::resize(mapCopy, mapCopy, newSize);
    cv::imshow("Path and Narrow Pairs Visualization", mapCopy);
    cv::waitKey(0);
}

void AlgorithmContour::savePicturePathAndContour(std::vector<Node2D> path2D) {
    float mul = 10; 
    cv::Mat mapCopy;
    cv::resize(this->gridMap, mapCopy, cv::Size(), mul, mul, cv::INTER_LINEAR);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
    cv::bitwise_not(mapCopy, mapCopy);
    
    for(uint i=0;i<path2D.size();i++){
        cv::circle(mapCopy, cv::Point(path2D[i].getFloatX() * mul, path2D[i].getFloatY() * mul), 2 * mul, cv::Scalar(0, 255, 0), -1);
    }
    for(auto & contour:this->refineContours){
        for(uint i=0;i<contour.size();i++){
            cv::circle(mapCopy, cv::Point(contour[i].x * mul, contour[i].y * mul), 2 * mul, cv::Scalar(255, 0, 0), -1);
        }
    }
    
    std::string packagePath = "/home/zly/plannerAll/catkin_path_planner";
    std::string imagePath = packagePath + "/picture/TPCAP_"+std::to_string(this->TPCAP_index)+"/PathAndNarrowPairsVisualization.jpg";
    std::string directoryPath = packagePath + "/picture/TPCAP_"+std::to_string(this->TPCAP_index);
    if (!boost::filesystem::exists(directoryPath)) {
        boost::filesystem::create_directories(directoryPath);
    }
    
    
    cv::flip(mapCopy, mapCopy, 0);
    
    cv::imwrite(imagePath, mapCopy);
}
void AlgorithmContour::savePicturePairAndKeyInformation(){
    float mul = 10; 
    cv::Mat mapCopy;
    cv::resize(this->gridMap, mapCopy, cv::Size(), mul, mul, cv::INTER_LINEAR);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
    cv::bitwise_not(mapCopy, mapCopy);
        for(uint i = 0; i < throughNarrowPairs.size(); i++) {
        std::pair<Node2D*, Node2D*> narrowPair = throughNarrowPairs[i];
        std::vector<Node2D> & path = throughNarrowPairsWaypoints[i];

        
        cv::circle(mapCopy, cv::Point(narrowPair.first->getFloatX() * mul, narrowPair.first->getFloatY() * mul), 3 * mul, cv::Scalar(0, 0, 255), -1);
        cv::circle(mapCopy, cv::Point(narrowPair.second->getFloatX() * mul, narrowPair.second->getFloatY() * mul), 3 * mul, cv::Scalar(255, 0, 0), -1);
    }
    for(keyInfoForThrouthNarrowPair* keyInfo:keyInfoForThrouthNarrowPairs) {
        cv::circle(mapCopy,cv::Point(keyInfo->firstBoundPoint->getFloatX()*mul,keyInfo->firstBoundPoint->getFloatY()*mul),2*mul,cv::Scalar(0, 255, 255), -1);
        cv::circle(mapCopy,cv::Point(keyInfo->secondBoundPoint->getFloatX()*mul,keyInfo->secondBoundPoint->getFloatY()*mul),2*mul,cv::Scalar(0, 255, 255), -1);
        cv::circle(mapCopy,cv::Point(keyInfo->firstBoundRealPoint.getFloatX()*mul,keyInfo->firstBoundRealPoint.getFloatY()*mul),2*mul,cv::Scalar(19, 69, 139), -1);
        cv::circle(mapCopy,cv::Point(keyInfo->secondBoundRealPoint.getFloatX()*mul,keyInfo->secondBoundRealPoint.getFloatY()*mul),2*mul,cv::Scalar(19, 69, 139), -1);
        cv::Point center(keyInfo->centerPoint->getFloatX()*mul, keyInfo->centerPoint->getFloatY()*mul);
        cv::Point centerVerticalEnd(center.x + Constants::each_meter_to_how_many_pixel * keyInfo->centerVerticalUnitVector.x * mul, center.y + Constants::each_meter_to_how_many_pixel * keyInfo->centerVerticalUnitVector.y * mul);
        cv::arrowedLine(mapCopy, center, centerVerticalEnd, cv::Scalar(255, 255, 0), int(1.5*mul));
    }
    std::string packagePath = "/home/zly/plannerAll/catkin_path_planner";
    std::string imagePath = packagePath + "/picture/TPCAP_"+std::to_string(this->TPCAP_index)+"/PKeyInformation.jpg";
    std::string directoryPath = packagePath + "/picture/TPCAP_"+std::to_string(this->TPCAP_index);
    if (!boost::filesystem::exists(directoryPath)) {
        boost::filesystem::create_directories(directoryPath);
    }
    
    cv::flip(mapCopy, mapCopy, 0);
    cv::imwrite(imagePath, mapCopy);
}
void AlgorithmContour::visualizekeyInfoForThrouthNarrowPair(std::pair<Node2D*, Node2D*> narrowPair, keyInfoForThrouthNarrowPair* keyInfo, const cv::Mat & gridMap) {
    cv::Mat mapCopy;
    float mul = AlgorithmContour::visualizeMultiplier;  
    cv::resize(gridMap, mapCopy, cv::Size(), mul, mul, cv::INTER_LINEAR);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
    
    cv::circle(mapCopy, cv::Point(narrowPair.first->getFloatX() * mul, narrowPair.first->getFloatY() * mul), 1.5, cv::Scalar(0, 0, 255), -1);
    cv::circle(mapCopy, cv::Point(narrowPair.second->getFloatX() * mul, narrowPair.second->getFloatY() * mul), 1.5, cv::Scalar(255, 0, 0), -1);
    cv::circle(mapCopy, cv::Point(keyInfo->centerPoint->getFloatX() * mul, keyInfo->centerPoint->getFloatY() * mul), 1.5, cv::Scalar(0, 255, 0), -1);
    cv::circle(mapCopy, cv::Point(keyInfo->firstBoundPoint->getFloatX() * mul, keyInfo->firstBoundPoint->getFloatY() * mul), 1.5, cv::Scalar(0, 255, 0), -1);
    cv::circle(mapCopy, cv::Point(keyInfo->secondBoundPoint->getFloatX() * mul, keyInfo->secondBoundPoint->getFloatY() * mul), 1.5, cv::Scalar(0, 255, 0), -1);

    
    cv::Point center(keyInfo->centerPoint->getFloatX() * mul, keyInfo->centerPoint->getFloatY() * mul);
    cv::Point centerVerticalEnd(center.x + Constants::each_meter_to_how_many_pixel * keyInfo->centerVerticalUnitVector.x * mul, center.y + Constants::each_meter_to_how_many_pixel * keyInfo->centerVerticalUnitVector.y * mul);
    cv::line(mapCopy, center, centerVerticalEnd, cv::Scalar(255, 255, 0), 2);
    
    cv::Size newSize(500 * gridMap.cols / gridMap.rows , 500 );
    cv::resize(mapCopy, mapCopy, newSize);

    
    cv::imshow("Key Information Visualization", mapCopy);
    cv::waitKey(0);
}




void drawPoints(const std::vector<Node3D>& points, cv::Mat& map, const cv::Scalar& color, float positionMul) {
    int arrowLength = 6 * positionMul;
    auto drawArrow = [&](const Node3D &startNode3D) {
      directionVector direction = directionVector::getUnitVectorFromNode3D(&startNode3D);
      cv::Point start(startNode3D.getX() * positionMul, startNode3D.getY() * positionMul);
      cv::Point end(start.x + direction.x * arrowLength, start.y + direction.y * arrowLength);
      cv::arrowedLine(map, start, end, color, 1 * positionMul, 8, 0, 0.3);
    };
    for (const auto& point : points) {
        drawArrow(point);
    }
}

std::vector<Node3D> AlgorithmContour::findNarrowPassSpace(CollisionDetection& configurationSpace,
    const directionVector& radiusVectorToYuanxin,
    const directionVector& tangentVector,Node2D* startPoint, int whetherReverse, int whetherCloseReverseGoal)
{
  if((whetherReverse!=1 && whetherReverse!=0) || (whetherCloseReverseGoal!=1 && whetherCloseReverseGoal!=0)){
    std::cerr<<"whetherReverse should be 1 or 0"<<std::endl;
    return std::vector<Node3D>();
  }

  float radius=Constants::minRadius;
  float maxRadius = Constants::maxRadus;
  std::vector<Node3D> finalCirclePath;
  std::vector<Node3D> qulifiedNode3DList20Degree;
  std::vector<Node3D> qulifiedNode3DList30Degree;
  std::vector<Node3D> qulifiedNode3DList45Degree;
  bool flagWhetherFindMinArc = false; 
  
  while (radius<=maxRadius)
  {
    float centerX = startPoint->getFloatX()+radiusVectorToYuanxin.x*radius;
    float centerY = startPoint->getFloatY()+radiusVectorToYuanxin.y*radius;

     
    Node2D* circleCenterPoint = new Node2D(centerX,centerY);
    
    float cross=tangentVector.x*radiusVectorToYuanxin.y-tangentVector.y*radiusVectorToYuanxin.x;
    if(cross>0){
      cross=1;
    }else cross=-1;

    float angleVehicleCurrent = atan2f(tangentVector.y,tangentVector.x);
    angleVehicleCurrent=Helper::normalizeHeadingRad(angleVehicleCurrent);
    float angleRelativeToCircleCurrent = 0;
    bool flag=true;
      
    finalCirclePath.clear();
    float currentRotatedAngel = 0;
    float deltaAngel = Constants::findNarrowSpaceMoveDistance/radius;
    float maxAngel = Constants::maxAngleRag * Constants::minRadius / radius;
    for(currentRotatedAngel=0;currentRotatedAngel<maxAngel;currentRotatedAngel+=deltaAngel){
      if(radius * currentRotatedAngel > Constants::maxNarrowSpaceArcLength){
        break;
      }
      angleRelativeToCircleCurrent = Helper::normalizeHeadingRad(angleVehicleCurrent +cross*M_PI/2 + M_PI )+ cross * deltaAngel;
      
      angleVehicleCurrent = Helper::normalizeHeadingRad(angleVehicleCurrent + cross * deltaAngel);
      float pointX=circleCenterPoint->getFloatX()+radius*cos(angleRelativeToCircleCurrent);
      float pointY=circleCenterPoint->getFloatY()+radius*sin(angleRelativeToCircleCurrent);
      Node3D NodeCurrentVehicle =  Node3D(pointX, pointY, Helper::normalizeHeadingRad(angleVehicleCurrent+M_PI*whetherReverse + M_PI*whetherCloseReverseGoal));
      
      if(!configurationSpace.isTraversableWithTolerance(&NodeCurrentVehicle,1)){
        flag=false;
        break;
      }else{
        finalCirclePath.push_back(NodeCurrentVehicle);
      }
    }
    if(whetherDeepDebug){
      cv::Mat mapCopy;
      float multiplier = AlgorithmContour::visualizeMultiplier;
      cv::resize(gridMap, mapCopy, cv::Size(), multiplier, multiplier, cv::INTER_NEAREST);
      cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
      cv::circle(mapCopy, cv::Point(circleCenterPoint->getFloatX()*multiplier, circleCenterPoint->getFloatY()*multiplier), 3, cv::Scalar(255, 0, 0), -1);
      cv::circle(mapCopy,cv::Point(startPoint->getFloatX()*multiplier,startPoint->getFloatY()*multiplier),3,cv::Scalar(0, 0, 255), -1);
      drawPoints(finalCirclePath, mapCopy, cv::Scalar(0, 255, 0),multiplier); 
      cv::imshow("Key Information Visualization", mapCopy);
      cv::waitKey(0);
    }
    if(flag){
      return finalCirclePath;
    }else{
      radius+=Constants::deltaRadius;
      if(currentRotatedAngel > (1.0/9.0)*M_PI && qulifiedNode3DList20Degree.size()==0){
        qulifiedNode3DList20Degree = finalCirclePath;
        flagWhetherFindMinArc = true;
      }
      if(currentRotatedAngel > 0.125*M_PI && qulifiedNode3DList30Degree.size()==0){
        qulifiedNode3DList30Degree = finalCirclePath;
      }
      if(currentRotatedAngel > 0.25*M_PI && qulifiedNode3DList45Degree.size()==0){
        qulifiedNode3DList45Degree = finalCirclePath;
      }
    }
  }
  if(flagWhetherFindMinArc == true){ 
    if(qulifiedNode3DList45Degree.size()!=0){
      return qulifiedNode3DList45Degree;
    }else if(qulifiedNode3DList30Degree.size()!=0){
      return qulifiedNode3DList30Degree;
    }else if(qulifiedNode3DList20Degree.size()!=0){
      return qulifiedNode3DList20Degree;
    }else{
      return finalCirclePath;
    }
  }else{
    std::cout << "进行直线搜索" << std::endl;
    float angleVehicleCurrent = atan2f(tangentVector.y,tangentVector.x);
    float offsetForStraightLine = Constants::width * 0.01;
    Node2D tempStart{startPoint->getFloatX(), startPoint->getFloatY()};
    directionVector offestDirectionVector = {-radiusVectorToYuanxin.x,-radiusVectorToYuanxin.y};
    bool successFlag = true;
    float allOffset = 0;
    while(true){
      successFlag = true;
      finalCirclePath.clear();
      for(float l = 0;l<=Constants::length/4;l+=Constants::findNarrowSpaceMoveDistance){
        float pointX=tempStart.getFloatX()+l*tangentVector.x;
        float pointY=tempStart.getFloatY()+l*tangentVector.y;
        Node3D NodeCurrentVehicle =  Node3D(pointX, pointY, Helper::normalizeHeadingRad(angleVehicleCurrent+M_PI*whetherReverse + M_PI*whetherCloseReverseGoal));
        if(!configurationSpace.isTraversableWithTolerance(&NodeCurrentVehicle,1)){
          std::cout << "直接使用直线生成的时候NarrowSpace的时候有误,增加偏移量" << std::endl;
          tempStart.setFloatx(tempStart.getFloatX()+offsetForStraightLine*offestDirectionVector.x);
          tempStart.setFloaty(tempStart.getFloatY()+offsetForStraightLine*offestDirectionVector.y);
          successFlag = false;
          allOffset += offsetForStraightLine;
          break;
        }
        finalCirclePath.push_back(NodeCurrentVehicle);
      }
      if(successFlag == true){
        break;
      }else{
        if(allOffset > Constants::width * 0.1){
          std::cout << "直接使用直线生成的时候NarrowSpace的时候有误,增加偏移量超过了最大值" << std::endl;
          break;
        }
      }
    }
    return finalCirclePath;
  }
  
}

void AlgorithmContour::findNarrowPassSpaceForAllPairs(CollisionDetection &configurationSpace,const Node3D & goal)
{
  for (keyInfoForThrouthNarrowPair* KIpair:keyInfoForThrouthNarrowPairs ) {
    if(two3DPointsWhetherCloseAndReverseDirection(configurationSpace,KIpair->centerPoint,KIpair->centerVerticalUnitVector,goal)){
      KIpair->whetherCloseReverseToGoal = true;
    }
    directionVector CVUR = KIpair->centerVerticalUnitVector.getReverseVector();
    int whetherCloseReverseToGoalSignals = KIpair->whetherCloseReverseToGoal?1:0;
    Node2D * realStartPoint = new Node2D(KIpair->firstBoundPoint->getFloatX(),
                                            KIpair->firstBoundPoint->getFloatY());
    if(whetherCloseReverseToGoalSignals == 0 && Constants::useRearAsCenter == 1){
      realStartPoint->setFloatx(realStartPoint->getFloatX()+CVUR.x*Constants::reverseBackDistance);
      realStartPoint->setFloaty(realStartPoint->getFloatY()+CVUR.y*Constants::reverseBackDistance);
    }
    KIpair->containingWaypointsFirstBPBackward=findNarrowPassSpace(configurationSpace,
        KIpair->wireUnitVector.getReverseVector(),CVUR,realStartPoint,1,whetherCloseReverseToGoalSignals);
    KIpair->firstBoundRealPoint = *realStartPoint;
    delete realStartPoint;
    realStartPoint = new Node2D(KIpair->secondBoundPoint->getFloatX(),
                                            KIpair->secondBoundPoint->getFloatY());
    if(whetherCloseReverseToGoalSignals == 0 && Constants::useRearAsCenter == 1){
      realStartPoint->setFloatx(realStartPoint->getFloatX()+CVUR.x*Constants::reverseBackDistance);
      realStartPoint->setFloaty(realStartPoint->getFloatY()+CVUR.y*Constants::reverseBackDistance);
    }
    KIpair->containingWaypointsSecondBPBackward=findNarrowPassSpace(configurationSpace,
        KIpair->wireUnitVector,CVUR,realStartPoint,1,whetherCloseReverseToGoalSignals);
    KIpair->secondBoundRealPoint = *realStartPoint;
    delete realStartPoint;
  }
}




void AlgorithmContour::visualizePassSpaceBoundaryForThroughNarrowPair(keyInfoForThrouthNarrowPair* keyInfo, const cv::Mat& gridMap) {
    cv::Mat mapCopy;
    float multiplier = AlgorithmContour::visualizeMultiplier;
    cv::resize(gridMap, mapCopy, cv::Size(), multiplier, multiplier, cv::INTER_NEAREST);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);

    
    drawPoints(keyInfo->containingWaypointsFirstBPBackward, mapCopy, cv::Scalar(0, 255, 0),multiplier); 
    drawPoints(keyInfo->containingWaypointsSecondBPBackward, mapCopy, cv::Scalar(255, 255, 0),multiplier); 
    cv::Size newSize(500 * gridMap.cols / gridMap.rows, 500);
    cv::resize(mapCopy, mapCopy, newSize);
    cv::imshow("Pass Space Boundary Visualization", mapCopy); 
    cv::waitKey(0); 
}
void AlgorithmContour::savePictureNarrowSpaceBoundary(){
    float mul = 10; 
    cv::Mat mapCopy;
    cv::resize(this->gridMap, mapCopy, cv::Size(), mul, mul, cv::INTER_LINEAR);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
    cv::bitwise_not(mapCopy, mapCopy);
    for(keyInfoForThrouthNarrowPair* keyInfo:keyInfoForThrouthNarrowPairs) {
        drawPoints(keyInfo->containingWaypointsFirstBPBackward, mapCopy, cv::Scalar(0, 0, 255),mul);
        drawPoints(keyInfo->containingWaypointsSecondBPBackward, mapCopy, cv::Scalar(255,0, 0),mul);
    }
    std::string packagePath = "/home/zly/plannerAll/catkin_path_planner";
    std::string imagePath = packagePath + "/picture/TPCAP_"+std::to_string(this->TPCAP_index)+"/PNarrowSpaceBoundaryVisualization.jpg";
    std::string directoryPath = packagePath + "/picture/TPCAP_"+std::to_string(this->TPCAP_index);
    if (!boost::filesystem::exists(directoryPath)) {
        boost::filesystem::create_directories(directoryPath);
    }
    
    
    cv::flip(mapCopy, mapCopy, 0);
    
    cv::imwrite(imagePath, mapCopy);
}


finalPassSpaceInOutSet AlgorithmContour::findNarrowPassSpaceInputSetOfNode3D(CollisionDetection& configurationSpace,keyInfoForThrouthNarrowPair* inputPair){
  finalPassSpaceInOutSet resultSet;
  uint size1 = inputPair->containingWaypointsFirstBPBackward.size();
  uint size2 = inputPair->containingWaypointsSecondBPBackward.size();
  uint minLength = std::min(size1,size2);
  std::vector<Node3D> inSetAllNode;
  int successIndex = 0;
  uint startIndex = std::floor(float(minLength)/1.5);
  
  for(uint i = startIndex; i < minLength; i++){
    Node3D* nodeFirst = &inputPair->containingWaypointsFirstBPBackward[size1-i-1];
    Node3D* nodeSecond = &inputPair->containingWaypointsSecondBPBackward[size2-i-1];
    std::vector<Node3D> resultVector = findInputSetOfNode3DByTwoVectorAndMiddleVerticalLine
      (*nodeFirst,*nodeSecond,*inputPair->centerPoint,inputPair->centerVerticalUnitVector,inputPair->whetherCloseReverseToGoal);
    bool flag = true;
    for(auto node:resultVector){
      if(!configurationSpace.isTraversableWithTolerance(&node,5)){
        std::cout << "在找到输入集的函数里面直线的插值不能通过！" << std::endl;
        flag = false;
        break;
      }
    }
    if(flag){
      inSetAllNode.insert(inSetAllNode.end(),resultVector.begin(),resultVector.end());
      successIndex++;
    }
    if(successIndex >=Constants::howManyLevelInputPick){
      break;
    }
  }
  if(inSetAllNode.size() == 0){
    std::cout << "插值集合是空，非常危险,必定出现问题！" << std::endl;
  }
  resultSet.inSet = inSetAllNode;
  return resultSet;
}


static inline std::vector<Node3D> interpolatePath(Node3D start, Node3D end, float gridSize) {
    std::vector<Node3D> path;

    float distance = std::sqrt(std::pow(end.getX() - start.getX(), 2) + std::pow(end.getY() - start.getY(), 2));
    int numberOfPoints = std::max(1, static_cast<int>(distance / gridSize * 1.1));
    float algelGap = end.getT() - start.getT(); 
    if (algelGap > M_PI) {
        algelGap -= 2 * M_PI;
    } else if (algelGap < -M_PI) {
        algelGap += 2 * M_PI;
    }
    for (int i = 0; i <= numberOfPoints; ++i) {
        float lerpFactor = static_cast<float>(i) / numberOfPoints;
        Node3D node;
        node.setX(start.getX() + lerpFactor * (end.getX() - start.getX()));
        node.setY(start.getY() + lerpFactor * (end.getY() - start.getY()));
        node.setT(Helper::normalizeHeadingRad(start.getT() + lerpFactor * algelGap));
        path.push_back(node);
    }

    return path;
}

std::vector<Node3D> AlgorithmContour::findInputSetOfNode3DByTwoVectorAndMiddleVerticalLine(const Node3D & firstPoint,const Node3D & secondPoint,
  const Node2D & middlePoint,const directionVector middleVerticalLine,int whetherCloseReverseGoal){
   Node3D intersection;

    
    Segment_2 segment(Point_2(firstPoint.getX(), firstPoint.getY()), Point_2(secondPoint.getX(), secondPoint.getY()));

    
    Line_2 middleLine = Line_2(Point_2(middlePoint.getX(), middlePoint.getY()), Point_2(middlePoint.getX() + middleVerticalLine.x, middlePoint.getY() + middleVerticalLine.y));

    
    auto result = CGAL::intersection(segment, middleLine);

    if (const Point_2* p = boost::get<Point_2>(&*result)) { 
      intersection.setX(static_cast<float>(CGAL::to_double(p->x())));
      intersection.setY(static_cast<float>(CGAL::to_double(p->y())));
    }
    float angelOfIntersection = atan2f(middleVerticalLine.y,middleVerticalLine.x);
    if(whetherCloseReverseGoal){
      angelOfIntersection = Helper::normalizeHeadingRad(angelOfIntersection + M_PI);
    }
    intersection.setT(Helper::normalizeHeadingRad(angelOfIntersection));
    if(this->whetherDeepDebug2){
      cv::Mat mapCopy;
      cv::cvtColor(this->gridMap, mapCopy, cv::COLOR_GRAY2BGR);
      float arrowLength = 10;
      auto drawArrow = [&](const Node3D &startNode3D) {
        directionVector direction = directionVector::getUnitVectorFromNode3D(&startNode3D);
        cv::Point start(startNode3D.getX(), startNode3D.getY());
        cv::Point end(start.x + direction.x * arrowLength, start.y + direction.y * arrowLength);
        cv::arrowedLine(mapCopy, start, end, cv::Scalar(0, 255, 0), 1, 8, 0, 0.3);
      };
      drawArrow(firstPoint);
      drawArrow(secondPoint);
      drawArrow(intersection);
      cv::circle(mapCopy, cv::Point(middlePoint.getX(), middlePoint.getY()), 2, cv::Scalar(0, 0, 255), -1);
      cv::imshow("visual the middle point", mapCopy); 
      cv::waitKey(0); 
    }
    std::vector<Node3D> firstPath = interpolatePath(firstPoint, intersection, Constants::interpolateGapForInputNarrowSpace);
    
    std::vector<Node3D> secondPath = interpolatePath(intersection,secondPoint, Constants::interpolateGapForInputNarrowSpace);
    std::vector<Node3D> resultVector;
    
    std::reverse(firstPath.begin(), firstPath.end());

    size_t firstPathIndex = 0;
    size_t secondPathIndex = 0;
    
    while (firstPathIndex < firstPath.size() || secondPathIndex < secondPath.size()) {
        if (firstPathIndex < firstPath.size()) {
            resultVector.push_back(firstPath[firstPathIndex++]);
        }
        if (secondPathIndex < secondPath.size()) {
            resultVector.push_back(secondPath[secondPathIndex++]);
        }
    }
    
    return resultVector;

}

void AlgorithmContour::visualizeInsetForThroughNarrowPairs(std::vector<finalPassSpaceInOutSet> finalPassSpaceInOutSets,const cv::Mat & gridMap){
  for( auto &inOutSet:finalPassSpaceInOutSets){
    cv::Mat mapCopy;
    int multiplier = AlgorithmContour::visualizeMultiplier;
    cv::resize(gridMap, mapCopy, cv::Size(), multiplier, multiplier, cv::INTER_NEAREST);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
    for (const auto& node : inOutSet.inSet) {
        
        cv::Point2f nodePos(node.getX() * multiplier, node.getY() * multiplier);

        
        float length = 10; 
        cv::Point2f direction(cos(node.getT()) * length, sin(node.getT()) * length);
        cv::Point2f endPoint = nodePos + direction;

        
        cv::arrowedLine(mapCopy, nodePos, endPoint, cv::Scalar(0, 0, 255), 1,8,0,0.3);
    }
    cv::Size newSize(500 * gridMap.cols / gridMap.rows, 500);
    cv::resize(mapCopy, mapCopy, newSize);
    cv::namedWindow("Pass Space Boundary Visualization", cv::WINDOW_NORMAL);
    cv::imshow("Pass Space Boundary Visualization", mapCopy); 
    cv::waitKey(0); 
  }
}
void AlgorithmContour::savePictureNarrowSpaceInputSet(){
    float mul = 10; 
    cv::Mat mapCopy;
    cv::resize(this->gridMap, mapCopy, cv::Size(), mul, mul, cv::INTER_LINEAR);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
    cv::bitwise_not(mapCopy, mapCopy);
    for( auto &inOutSet:finalPassSpaceInOutSets){
      for (const auto& node : inOutSet.inSet) {
          
          cv::Point2f nodePos(node.getX() * mul, node.getY() * mul);

          
          float length = 8 * mul; 
          cv::Point2f direction(cos(node.getT()) * length, sin(node.getT()) * length);
          cv::Point2f endPoint = nodePos + direction;
          
          cv::arrowedLine(mapCopy, nodePos, endPoint, cv::Scalar(0, 0, 255), 0.8 * mul,8,0,0.3);
      }
    }
    std::string packagePath = "/home/zly/plannerAll/catkin_path_planner";
    std::string imagePath = packagePath + "/picture/TPCAP_"+std::to_string(this->TPCAP_index)+"/NarrowSpaceInputSetVisualization.jpg";
    std::string directoryPath = packagePath + "/picture/TPCAP_"+std::to_string(this->TPCAP_index);
    if (!boost::filesystem::exists(directoryPath)) {
        boost::filesystem::create_directories(directoryPath);
    }
    
    
    cv::flip(mapCopy, mapCopy, 0);
    
    cv::imwrite(imagePath, mapCopy);
}

void AlgorithmContour::findNarrowPassSpaceInputSetOfNode3DForAllPairs(CollisionDetection& configurationSpace){
  for (keyInfoForThrouthNarrowPair* KIpair:keyInfoForThrouthNarrowPairs ) {
    this->finalPassSpaceInOutSets.push_back(findNarrowPassSpaceInputSetOfNode3D(configurationSpace,KIpair));
  }
}
























