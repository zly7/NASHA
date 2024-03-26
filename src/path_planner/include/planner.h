#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"
#include "algorithmcontour.h"
#include "algorithmsplit.h"
#include "std_msgs/Int32.h"
#include <std_msgs/String.h>
#include "rrtalgorithm.h"

namespace HybridAStar {   

class Planner {
 public:
  
  Planner();

  
  void initializeLookups();

  
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);

  
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

  
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

  
  void plan();

  void timerForAutoTestCallback(const ros::TimerEvent& event);

 private:
  
  ros::NodeHandle n;
  
  ros::Publisher pubStart;
  
  ros::Publisher pubNotification;
  
  ros::Publisher pubAlgorithm;
  ros::Subscriber subMap;
  
  ros::Subscriber subGoal;
  
  ros::Subscriber subStart;
  
  tf::TransformListener listener;
  
  tf::StampedTransform transform;
  
  Path path;
  
  Smoother smoother;
  
  Path smoothedPath = Path(true);
  
  Visualize visualization;
  
  CollisionDetection configurationSpace;
  
  DynamicVoronoi voronoiDiagram;
  
  nav_msgs::OccupancyGrid::Ptr grid;
  
  geometry_msgs::PoseWithCovarianceStamped start;
  
  geometry_msgs::PoseStamped goal;
  
  bool validStart = false;
  
  bool validGoal = false;
  
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];

  int point_index=0;

  ros::Timer timerForAutoTest;
  bool whetherStartTest = false;
};
}
#endif 
