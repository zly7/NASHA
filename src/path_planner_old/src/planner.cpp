#define DEBUG_SHOW_INSTANT_ALGORITHMCONTOUR
#define DEBUG_SAVE_PICTURE
#include "planner.h"
using namespace HybridAStar;
Planner::Planner() {
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  pubAlgorithm = n.advertise<std_msgs::String>("/algForHA", 1);
  pubNotification = n.advertise<std_msgs::Int32>("start_notification", 5);
  if(Constants::useAutoTest){
    timerForAutoTest = n.createTimer(ros::Duration(1.0), &Planner::timerForAutoTestCallback, this);
  }
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }
  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
  Node3D::initializeVectorsForForward();
};
void Planner::timerForAutoTestCallback(const ros::TimerEvent& event){
  if(!whetherStartTest){
    std_msgs::Int32 msg;
    msg.data = point_index;  
    pubNotification.publish(msg);
  }
}
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }
  Lookup::collisionLookup(collisionLookup);
}
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {  
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }
  grid = map;
  configurationSpace.updateGrid(map);
  int height = map->info.height;
  int width = map->info.width;
  Node3D::widthForMap = width;
  Node2D::widthForMap = width;
  Node3D::heightForMap = height;
  std::cout << "when setting map in Planner, height: " << height << " width: " << width << std::endl;
  bool** binMap;
  binMap = new bool*[width];
  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      std::cout  << map->data[y * width + x];
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
    std::cout << std::endl;
  }
  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);
    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      validStart = true;
    } else  {
      validStart = false;
    }
    plan();
  }else{
    std_msgs::Int32 msg;
    msg.data = point_index;  
    pubNotification.publish(msg);
  }
}
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  whetherStartTest = true;
  float x = initial->pose.pose.position.x / Constants::cellSize;  
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();
  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;
    if (Constants::manual) { plan();}
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  whetherStartTest = true;
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);
  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;
    if (Constants::manual) { plan();}
  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}
void Planner::plan() { 
  std_msgs::String msgAlg;
  msgAlg.data = Constants::algorithm;
  pubAlgorithm.publish(msgAlg);
  if (validStart && validGoal) {
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    #ifdef DEBUG_MANUAL_START_GOAL
    x = 173;
    y = 145;
    t = Helper::normalizeHeadingRad(0);
    #endif
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    const Node2D nGoal2D(x, y, 0, 0, nullptr);
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    #ifdef DEBUG_MANUAL_START_GOAL
    x = 65;
    y = 200;
    t = Helper::normalizeHeadingRad(4.71);
    #endif
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    Node2D nStart2D(x, y, 0, 0, nullptr);
    ros::Time t0 = ros::Time::now();
    visualization.clear();
    path.clear();
    smoothedPath.clear();
    smoother.clear();
    std_msgs::Int32 msg1;
    msg1.data = -1;  
    pubNotification.publish(msg1);
    if(!configurationSpace.isTraversable(&nStart)){
      std::cout<<"起始点不能通行"<<std::endl;
    }
    if (!configurationSpace.isTraversable(&nGoal)) {
      std::cout<<"目标点不能通行"<<std::endl;
    }
    if(Constants::algorithm == "split_hybrid_astar"){
      auto startTime = std::chrono::high_resolution_clock::now();
      Node2D* nodes2D = new Node2D[width * height]();
      Node2D* nSolution2D = Algorithm::aStar2D(nStart2D, nGoal2D, nodes2D, width, height, configurationSpace, visualization);
      smoother.tracePath2D(nSolution2D);
      std::vector<Node2D> path2D=smoother.getPath2D();
      std::reverse(path2D.begin(),path2D.end());
      float deltaL=0.1 * Constants::each_meter_to_how_many_pixel;
      AlgorithmSplit::node2DToBox(path2D,width,height,configurationSpace,deltaL);
      path.update2DPath(path2D);
      path.publishPath2DNodes();
      path.publishPathBoxes();
      float threshold=Constants::width * 1.4;
      std::vector<Node3D> nodeBou=AlgorithmSplit::findBou(nStart,nGoal,path2D,threshold); 
      std::vector<multiGoalSet3D> multiGoalsBou;
      for(size_t k = 0; k<nodeBou.size();k++){
        if(k==0 || k==nodeBou.size()-1){
          multiGoalSet3D goals = multiGoalSet3D();
          goals.addGoal(nodeBou[k]);
          multiGoalsBou.push_back(goals);
        }else{
          multiGoalSet3D goals = multiGoalSet3D::fuzzyOneNodeToSetForSplitAstar(configurationSpace,nodeBou[k]);
          if(goals.goals.size()>0){
            multiGoalsBou.push_back(goals);
          }
        }
      }
      std::cout<<"findBou Finished!"<<std::endl;
      Node3D tempSolution=nStart;
      Node3D * nSolution = nullptr;
      delete [] nodes2D;
      for(size_t i = 1; i<nodeBou.size() ; i++){
        Node3D* nodes3D = new Node3D[length]();
        Node2D* nodes2D = new Node2D[width * height]();
        nSolution = Algorithm::hybridAStarMultiGoals(tempSolution, multiGoalsBou[i], nodes3D, nodes2D, width, height, 
              configurationSpace, dubinsLookup, visualization,Tolerance());
        smoother.tracePathAndReverse(nSolution);
        if(nSolution!=nullptr){
          tempSolution = *nSolution;
        }else{
          std::cout<<"Split A-star第"<<i<<"个目标点搜索失败,出现重大问题"<<std::endl;
        }
        delete [] nodes3D;
        delete [] nodes2D;
      }
      this->path.updatePath(smoother.getPath());
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "split HA搜索消耗时间: "<< duration.count() << "  ms" << std::endl;
    }else if(Constants::algorithm == "hybrid_astar"){
      auto startTime = std::chrono::high_resolution_clock::now();
      Node3D* nodes3D = new Node3D[length]();
      Node2D* nodes2D = new Node2D[width * height]();
      Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization,Tolerance());
      smoother.tracePath(nSolution);
      path.updatePath(smoother.getPath());
      smoothedPath.updatePath(smoother.getPath());
      delete [] nodes3D;
      delete [] nodes2D;
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "HA搜索消耗时间: "<< duration.count() << "  ms" << std::endl;
    }else if(Constants::algorithm == "contour_hybrid_astar"){
      Node2D* nodes2D = new Node2D[width * height]();
      auto startTime = std::chrono::high_resolution_clock::now();
      Node2D* nSolution2D = Algorithm::aStar2D(nStart2D, nGoal2D, nodes2D, width, height, configurationSpace, visualization);
      smoother.tracePath2D(nSolution2D);
      std::vector<Node2D> path2D=smoother.getPath2D();
      std::reverse(path2D.begin(),path2D.end());
      path.update2DPath(path2D);
      path.publishPath2DNodes();
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "2D A* 使用时间: "<< duration.count() << "  ms" << std::endl;
      startTime = std::chrono::high_resolution_clock::now();
      AlgorithmContour algorithmContour;
      algorithmContour.findContour(grid);     
      algorithmContour.findNarrowContourPair();
      algorithmContour.findThroughNarrowContourPair(path2D);
      algorithmContour.sortThroughNarrowPairsWaypoints();
      stop  = std::chrono::high_resolution_clock::now();
      duration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      auto tempDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "寻找到狭窄点对消耗时间: "<< tempDuration.count() << "  ms" << std::endl;
      #ifdef DEBUG_VISUAL_ALGORITHMCONTOUR
      for(uint i = 0;i<algorithmContour.throughNarrowPairs.size();i++){
        AlgorithmContour::visualizePathAndItNarrowPair(algorithmContour.throughNarrowPairsWaypoints[i],
              algorithmContour.throughNarrowPairs[i],algorithmContour.gridMap);
      }
      #endif
      if(Constants::saveMapCsv){
        algorithmContour.saveMapCsv(nStart,nGoal);
      }
      startTime = std::chrono::high_resolution_clock::now();
      algorithmContour.findKeyInformationForThrouthNarrowPairs();
      algorithmContour.findNarrowPassSpaceForAllPairs(configurationSpace,nGoal);
      algorithmContour.findNarrowPassSpaceInputSetOfNode3DForAllPairs(configurationSpace);
      stop  = std::chrono::high_resolution_clock::now();
      duration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      tempDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "寻找到狭窄点对的可行域消耗时间: "<< tempDuration.count() << "  ms" << std::endl;
      #ifdef DEBUG_VISUAL_ALGORITHMCONTOUR
      for(uint i = 0;i<algorithmContour.throughNarrowPairs.size();i++){
        AlgorithmContour::visualizekeyInfoForThrouthNarrowPair(algorithmContour.throughNarrowPairs[i],
              algorithmContour.keyInfoForThrouthNarrowPairs[i],algorithmContour.gridMap);
      }
      for(uint i = 0;i<algorithmContour.throughNarrowPairs.size();i++){
        AlgorithmContour::visualizePassSpaceBoundaryForThroughNarrowPair(algorithmContour.keyInfoForThrouthNarrowPairs[i],algorithmContour.gridMap);
      }
      AlgorithmContour::visualizeInsetForThroughNarrowPairs(algorithmContour.finalPassSpaceInOutSets,algorithmContour.gridMap);
      #endif
      #ifdef  DEBUG_SAVE_PICTURE
      algorithmContour.savePicturePathAndContour(path2D);
      algorithmContour.savePicturePairAndKeyInformation();
      algorithmContour.savePictureNarrowSpaceBoundary();
      algorithmContour.savePictureNarrowSpaceInputSet();
      #endif  
      startTime = std::chrono::high_resolution_clock::now();
      Node3D tempStart;
      for(uint i = 0;i<algorithmContour.finalPassSpaceInOutSets.size();i++){
        if(i==0){
          tempStart = nStart;
        }
        Node3D* nSolution1= nullptr;
        Node3D* nodes3D = new Node3D[length]();
        Node2D* nodes2DSplitSearch = new Node2D[width * height]();
        multiGoalSet3D goals = multiGoalSet3D();
        goals.addGoals(algorithmContour.finalPassSpaceInOutSets[i].inSet);
        nSolution1 = Algorithm::hybridAStarMultiGoals(tempStart, goals, nodes3D, nodes2DSplitSearch, width, height, 
            configurationSpace, dubinsLookup, visualization,Tolerance(Constants::tolerance,Constants::deltaHeadingRad));
        smoother.tracePathAndReverse(nSolution1);
        if(!Constants::whetherSplitSearch){
          tempStart = *nSolution1;
        }
        #ifdef DEBUG_SHOW_INSTANT_ALGORITHMCONTOUR
        auto StartTimeSub = std::chrono::high_resolution_clock::now();
        smoothedPath.updatePath(smoother.getPath());
        smoothedPath.publishPath();
        smoothedPath.publishPathNodes();
        smoothedPath.publishPathVehicles();
        auto stopSub = std::chrono::high_resolution_clock::now();
        auto durationSub = std::chrono::duration_cast<std::chrono::milliseconds>(stopSub - StartTimeSub);
        duration -= durationSub;
        #endif
        #ifdef DEBUG_VISUAL_COSTMAP
        visualization.publishNode3DCosts(nodes3D, width, height, depth);
        visualization.publishNode2DCosts(nodes2DSplitSearch, width, height);
        #endif
        std::cout<<"第"<<i<<"次搜索结束"<<std::endl;
      }
      Node3D* nodes3D = new Node3D[length]();
      Node2D* nodes2DSplitSearch = new Node2D[width * height]();
      Node3D* tempGoal;
      #ifdef DEBUG_SHOW_INSTANT_ALGORITHMCONTOUR
        if(!configurationSpace.isTraversable(&nGoal)){
          std::cout<<"目标设置有误，无法搜索"<<std::endl;
        }
      #endif
      if (Constants::whetherFuzzyGoal) {
          multiGoalSet3D goalsMulFinal = multiGoalSet3D::fuzzyOneNodeToSet(configurationSpace,nGoal);
          auto startGoal = algorithmContour.finalPassSpaceInOutSets.size() > 0 ? smoother.getPath().back(): nStart;
          tempGoal = Algorithm::hybridAStarMultiGoals(startGoal, goalsMulFinal, nodes3D, nodes2DSplitSearch, width, height, configurationSpace, dubinsLookup, visualization,Tolerance());
      } else {
          auto startGoal = algorithmContour.finalPassSpaceInOutSets.size() > 0 ? smoother.getPath().back() : nStart;
          tempGoal = Algorithm::hybridAStar(startGoal, nGoal, nodes3D, nodes2DSplitSearch, width, height, configurationSpace, dubinsLookup, visualization,Tolerance());
      }
      if(tempGoal!=nullptr ){
        smoother.tracePathAndReverse(tempGoal);
      }else{
        std::cout<<"在进入最后一段搜索的时候tempgoal是nullptr"<<std::endl;
      }
      if(Constants::whetherFuzzyGoal){
        std::vector<Node3D> interpolatedPath = Node3D::interpolateDirect(*tempGoal,nGoal,Constants::arcLengthForAstarSuccessor);
        smoother.getPathNotConst().insert(smoother.getPathNotConst().end(), interpolatedPath.begin(), interpolatedPath.end());
      }
      this->path.updatePath(smoother.getPath());
      delete [] nodes3D;
      delete [] nodes2DSplitSearch;
      delete [] nodes2D;
      stop = std::chrono::high_resolution_clock::now();
      duration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      tempDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "多次HA搜索消耗时间: "<< tempDuration.count() << "  ms" << std::endl;
      std::cout << "AlgorithmContour总花时间: "<< duration.count() << "  ms" << std::endl;
    }else if(Constants::algorithm == "rrt"){
      RrtAlgorithm rrtAlgorithm;
      Node2D* nodes2D = new Node2D[width * height]();
      Node2D* nSolution2D = rrtAlgorithm.rrtStar(nStart2D, nGoal2D, nodes2D, width, height, configurationSpace, visualization,Constants::iterations);
      smoother.tracePath2D(nSolution2D);
      std::vector<Node2D> path2D=smoother.getPath2D();
      std::reverse(path2D.begin(),path2D.end());
      path.update2DPath(path2D);
      path.publishPath2DNodes();
      delete [] nodes2D;
    }
    else{
      std::cout<<"algorithm error"<<std::endl;
    }
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "整个流程连带可视化和人关闭窗口时间 in ms: " << d * 1000 << std::endl;
    validStart=false;
    validGoal=false;
    point_index++;
    std_msgs::Int32 msg2;
    msg2.data = point_index;  
    pubNotification.publish(msg2);
    std::cout<<"point_index: "<<point_index<<std::endl;
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    std_msgs::Int32 msg;
    msg.data = -2;  
    pubNotification.publish(msg);
  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
