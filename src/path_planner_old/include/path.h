#ifndef PATH_H
#define PATH_H
#include <iostream>
#include <cstring>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "node3d.h"
#include "node2d.h"
#include "constants.h"
#include "helper.h"
namespace HybridAStar {
class Path {
 public:
  Path(bool smoothed = false) {
    std::string pathTopic = "/path";
    std::string pathNodesTopic = "/pathNodes";
    std::string pathVehicleTopic = "/pathVehicle";
    std::string path2DNodesTopic = "/path2DNodes";
    std::string pathBoxesTopic = "/pathBoxes";
    if (smoothed) {
      pathTopic = "/sPath";
      pathNodesTopic = "/sPathNodes";
      pathVehicleTopic = "/sPathVehicle";
      path2DNodesTopic = "/sPath2DNodes";
      pathBoxesTopic = "/sPathBoxes";
      this->smoothed = smoothed;
    }
    pubPath = n.advertise<nav_msgs::Path>(pathTopic, 1);
    pubPathNodes = n.advertise<visualization_msgs::MarkerArray>(pathNodesTopic, 1);
    pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>(pathVehicleTopic, 1);
    pubPath2DNodes = n.advertise<visualization_msgs::MarkerArray>(path2DNodesTopic, 1);
    pubPathBoxes = n.advertise<visualization_msgs::MarkerArray>(pathBoxesTopic, 1);
    path.header.frame_id = "path";
  }
  void updatePathFromK(const std::vector<Node3D> &nodePath, int k);
  void updatePath(const std::vector<Node3D> &nodePath);
  void update2DPath(const std::vector<Node2D> &nodePath);
  void addSegment(const Node3D& node);
  void addNode(const Node3D& node, int i);
  void add2DNode(const Node2D& node, int i);
  void add2DBox(const Node2D& node, int i);
  void addVehicle(const Node3D& node, int i);
  void tempUpdatePathNode(const std::vector<Node3D> &nodePath);
  void clear();
  void publishPath() { pubPath.publish(path); }
  void publishPathNodes() { pubPathNodes.publish(pathNodes); }
  void publishPathVehicles() { pubPathVehicles.publish(pathVehicles); }
  void publishPath2DNodes() { pubPath2DNodes.publish(path2DNodes); }
  void publishPathBoxes() { pubPathBoxes.publish(pathBoxes); }
 private:
  ros::NodeHandle n;
  ros::Publisher pubPath;
  ros::Publisher pubPathNodes;
  ros::Publisher pubPathVehicles;
  ros::Publisher pubPath2DNodes;
  ros::Publisher pubPathBoxes;
  nav_msgs::Path path;
  visualization_msgs::MarkerArray pathNodes;
  visualization_msgs::MarkerArray pathVehicles;
  visualization_msgs::MarkerArray path2DNodes;
  visualization_msgs::MarkerArray pathBoxes;
  bool smoothed = false;
};
}
#endif 
