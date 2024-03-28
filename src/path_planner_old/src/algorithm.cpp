#include "algorithm.h"
typedef CGAL::Exact_circular_kernel_2 Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Line_2 Line_2;
typedef Kernel::Circle_2 Circle_2;
typedef Kernel::Circular_arc_point_2   Circular_arc_point_2;
typedef Kernel::Circular_arc_2 Circular_arc_2;
typedef Kernel::Vector_2               Vector_2;
using namespace HybridAStar;
float aStar2DForUpdateHValueOf3DNode(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
void updateHFor3DNode(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
double measureTime(std::function<void()> func) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    return elapsed.count();
}
struct CompareNodes {
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};
Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization,Tolerance tol) 
                              {
  multiGoalSet3D goalSet;
  goalSet.addGoal(goal);
  return hybridAStarMultiGoals(start, goalSet, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization, tol);
                               }
Node3D* Algorithm::hybridAStarMultiGoals(Node3D& start,
                               multiGoalSet3D& goalSet,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization,Tolerance tol) {
  if(Constants::visualizationStartAndGoal){
    visualization.publishNode3DStartAndGoal(start, goalSet.virtualCenterNode3D);
  }
  int iPred, iSucc;
  float newG;
  int dir = Constants::reverse ? 2*Node3D::dir : Node3D::dir;
  int iterations = 0;
  ros::Duration d(0.003);
  start.setPred(nullptr);
  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;
  start.open();
  O.push(&start);
  iPred = start.getIdx();
  nodes3D[iPred] = start;
  std::cout<<"start "<<start.getX()<<" "<<start.getY()<< " " << start.getT()<<std::endl;
  Node3D* nPred;
  Node3D* nSucc;
  #ifdef DEBUG_TIME_ASTAR3D
  int currentLoop = 0;
  double allRuningTime = 0;
  double allRuningTimeDubinsShot = 0;
  double allRuningTimeUpdateH = 0;
  double allRuningTimeisTraversal = 0;
  #endif
  while (!O.empty()) {
    #ifdef DEBUG_TIME_ASTAR3D
    auto startLoopTime = std::chrono::high_resolution_clock::now();
    #endif
    nPred = O.top();
    iPred = nPred->getIdx();
    iterations++;
    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }
    if (nodes3D[iPred].isClosed()) {
      O.pop();
      continue;
    }
    else if (nodes3D[iPred].isOpen()) {
      nodes3D[iPred].close();
      O.pop();
      if(nPred->get2DDistance(goalSet.virtualCenterNode3D) < Constants::length){
        for(auto &goal : goalSet.goals){
          if ((*nPred).isEqualWithTolerance(goal,tol)) {
            std::cout<<"总迭代次数: "<< iterations<<std::endl;
            std::cout<<"npred "<<nPred->getX()<<" "<<nPred->getY()<< " " << nPred->getT()<<std::endl;
            std::cout<<"goal "<<goal.getX()<<" "<<goal.getY() << ""<< goal.getT()<<std::endl; 
            std::cout<<"Hybrid3D结束搜索总的搜索次数: "<<iterations<<std::endl;
            return nPred;
          }
        }
      }
      if(iterations > Constants::iterations){
        std::cout<<"到达了最长的搜索迭代次数,未能找到目标点"<<std::endl;
        return nullptr;
      }
      if(((iterations+1) / Constants::iterationsToPrint)!= iterations / Constants::iterationsToPrint){
        std::cout<<"Hybrid3D已经过了 "<<iterations << " 次搜索未搜索到结果"<<std::endl;
      }
      if (Constants::useArcShot && nPred->isInArcRange(goalSet.virtualCenterNode3D)){
        for (auto &goal : goalSet.goals){
          nSucc = ArcShot(*nPred, goal, configurationSpace);
          if (nSucc != nullptr && *nSucc == goal) {  
          std::cout << "通过ArcShot 命中结束点" << std::endl;
          std::cout<< "start "<< start.getX()<<" "<<start.getY()<<std::endl;
          std::cout<<"nSucc "<<nSucc->getX()<<" "<<nSucc->getY()<<std::endl;
          std::cout<<"goal "<<goal.getX()<<" "<<goal.getY()<<std::endl; 
          return nSucc;
          }
        }
      }
      if (Constants::dubinsShot && nPred->isInRange(goalSet.virtualCenterNode3D) ) {
        #ifdef DEBUG_TIME_ASTAR3D
        auto startDubinsShotTime = std::chrono::high_resolution_clock::now();
        #endif
        if(Constants::randomDubinsShot){
          Node3D randomDubinNode = goalSet.getRandomGoal();
          nSucc = dubinsShot(*nPred, randomDubinNode, configurationSpace);
          if (nSucc != nullptr && *nSucc == randomDubinNode) { 
            std::cout << "通过dubinShot 命中结束点" << std::endl;
            std::cout<<"nSucc "<<nSucc->getX()<<" "<<nSucc->getY()<<std::endl;
            std::cout<<"goal "<<randomDubinNode.getX()<<" "<<randomDubinNode.getY()<<std::endl; 
            return nSucc;
          }
        }else{
          for (auto &goal : goalSet.goals){
            nSucc = dubinsShot(*nPred, goal, configurationSpace);
            if (nSucc != nullptr && *nSucc == goal) {  
              std::cout << "通过直接搜索 命中结束点" << std::endl;
              std::cout<<"nSucc "<<nSucc->getX()<<" "<<nSucc->getY()<<std::endl;
              std::cout<<"goal "<<goal.getX()<<" "<<goal.getY()<<std::endl; 
              return nSucc;
            }
          }
        }
        #ifdef DEBUG_TIME_ASTAR3D
        auto stopDubinsShotTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = stopDubinsShotTime - startDubinsShotTime;
        allRuningTimeDubinsShot += elapsed.count();
        #endif
      }
      for (int i = 0; i < dir; i++) {  
        nSucc = nPred->createSuccessor(i);
        iSucc = nSucc->getIdx();
        #ifdef DEBUG_TIME_ASTAR3D
        auto startisTraversableTime = std::chrono::high_resolution_clock::now();
        #endif
        bool isTraversable = configurationSpace.isTraversable(nSucc);
        #ifdef DEBUG_TIME_ASTAR3D
        auto stopisTraversableTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedisTraversable = stopisTraversableTime - startisTraversableTime;
        allRuningTimeisTraversal += elapsedisTraversable.count();
        #endif
        if (nSucc->isOnGrid(width, height) && isTraversable) {
          if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {
            nSucc->updateG();
            newG = nSucc->getG();
            if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {
              #ifdef DEBUG_TIME_ASTAR3D
              auto startupdateHTime = std::chrono::high_resolution_clock::now();
              #endif
              updateHFor3DNode(*nSucc, goalSet.virtualCenterNode3D, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);  
              #ifdef DEBUG_TIME_ASTAR3D
              auto stopupdateHTime = std::chrono::high_resolution_clock::now();
              std::chrono::duration<double> elapsedupdateH = stopupdateHTime - startupdateHTime;
              std::cout << "elapsedupdateH: " << elapsedupdateH.count() << std::endl;
              allRuningTimeUpdateH += elapsedupdateH.count();
              #endif
              if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                delete nSucc;
                continue;
              }
              else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                nSucc->setPred(nPred->getPred());
              }
              if (nSucc->getPred() == nSucc) {
                std::cout << "looping";
              }
              nSucc->open();
              nodes3D[iSucc] = *nSucc;
              O.push(&nodes3D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        } else { delete nSucc; }
      }
    }
    #ifdef DEBUG_TIME_ASTAR3D
    auto stopLoopTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stopLoopTime - startLoopTime;
    allRuningTime += elapsed.count();
    if(currentLoop % 10 == 0) {
      std::cout << "currentLoop: "<< currentLoop<<" all Runing time: " << allRuningTime << std::endl;
      std::cout << "currentLoop: "<< currentLoop<<" all Runing time DubinsShot: " << allRuningTimeDubinsShot << std::endl;
      std::cout << "currentLoop: "<< currentLoop<<" all Runing time updateH: " << allRuningTimeUpdateH << std::endl;
      std::cout << "currentLoop: "<< currentLoop<<" all Runing time isTraversal: " << allRuningTimeisTraversal << std::endl;
    }
    currentLoop++;
    #endif
  }
  if (O.empty()) {
    return nullptr;
  }
  return nullptr;
}
float aStar2DForUpdateHValueOf3DNode(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {
  int iPred, iSucc;
  float newG;
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }
  ros::Duration d(0.001);
  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  start.updateH(goal);
  start.open();
  O.push(&start);
  iPred = start.getIdx();
  nodes2D[iPred] = start;
  Node2D* nPred;
  Node2D* nSucc;
  while (!O.empty()) {
    nPred = O.top();
    iPred = nPred->getIdx();
    if (nodes2D[iPred].isClosed()) {
      O.pop();
      continue;
    }
    else if (nodes2D[iPred].isOpen()) {
      nodes2D[iPred].close();
      nodes2D[iPred].discover();
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
      }
      O.pop();
      if (*nPred == goal) {
        return nPred->getG();
      }
      else {
        for (int i = 0; i < Node2D::dir; i++) {
          nSucc = nPred->createSuccessor(i); 
          iSucc = nSucc->getIdx();
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isObstacleWidthCircle(nSucc) && !nodes2D[iSucc].isClosed()) {
            nSucc->updateG(); 
            newG = nSucc->getG();
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              nSucc->updateH(goal);
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }
  return 1000;
}
Node2D* Algorithm::aStar2D(Node2D& start,
            const Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {
  int iPred, iSucc;
  float newG;
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }
  ros::Duration d(0.001);
  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  start.updateH(goal);
  start.open();
  O.push(&start);
  iPred = start.getIdx();
  nodes2D[iPred] = start;
  Node2D* nPred;
  Node2D* nSucc;
  while (!O.empty()) {
    nPred = O.top();
    iPred = nPred->getIdx();
    if (nodes2D[iPred].isClosed()) {
      O.pop();
      continue;
    }
    else if (nodes2D[iPred].isOpen()) {
      nodes2D[iPred].close();
      nodes2D[iPred].discover();
      if (Constants::visualization2D && false) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        d.sleep();
      }
      O.pop();
      if (*nPred == goal) {
        return nPred;
      }
      else {
        for (int i = 0; i < Node2D::dir; i++) {
          nSucc = nPred->createSuccessor(i);
          iSucc = nSucc->getIdx();
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            nSucc->updateG(); 
            newG = nSucc->getG();
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              nSucc->updateH(goal);
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }
  return nullptr;
}
void updateHFor3DNode(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, 
  CollisionDetection& configurationSpace, Visualize& visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;
  #ifdef DEBUG_TIME_UPDATEH
  double runAllTime = 0;
  double runReedSheppTime = 0;
  double runAstarForH = 0;
  auto startAllTime = std::chrono::high_resolution_clock::now();
  #endif
  if (Constants::dubins && Constants::useDubinReedSheepHeuristic) {
    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }
  #ifdef DEBUG_TIME_UPDATEH
  auto startReedSheppTime = std::chrono::high_resolution_clock::now();
  #endif
  if (Constants::reverse && !Constants::dubins && Constants::useDubinReedSheepHeuristic) {
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
  }
  #ifdef DEBUG_TIME_UPDATEH
  auto stopReedSheppTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedReedShepp = stopReedSheppTime - startReedSheppTime;
  runReedSheppTime += elapsedReedShepp.count();
  #endif
  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    #ifdef DEBUG_TIME_UPDATEH
    auto startAstarForHTime = std::chrono::high_resolution_clock::now();
    #endif
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(
        aStar2DForUpdateHValueOf3DNode( goal2d,start2d, nodes2D, width, height, configurationSpace, visualization));
    #ifdef DEBUG_TIME_UPDATEH
    auto stopAstarForHTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedAstarForH = stopAstarForHTime - startAstarForHTime;
    runAstarForH += elapsedAstarForH.count();
    #endif
  }
  if (Constants::twoD) {
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
  }
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
  #ifdef DEBUG_TIME_UPDATEH
  auto stopAllTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedAll = stopAllTime - startAllTime;
  runAllTime += elapsedAll.count();
  std::cout << "runReedSheppTime: " << runReedSheppTime << std::endl;
  std::cout << "runAstarForH: " << runAstarForH << std::endl;
  std::cout << "runAllTime: " << runAllTime << std::endl;
  #endif
}
Node3D* Algorithm::dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  double q0[] = { start.getX(), start.getY(), start.getT() };
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  DubinsPath path;
  dubins_init(q0, q1, Constants::r, &path);
  int primToInherit = start.getPrim();
  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);
  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];
  x += Constants::dubinsStepSize;
  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));
    dubinsNodes[i].setPrim(primToInherit);
    if ( dubinsNodes[i].isOnGrid() &&configurationSpace.isTraversable(&dubinsNodes[i])) {
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }
      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }
      x += Constants::dubinsStepSize;
      i++;
    } else {
      delete [] dubinsNodes;
      return nullptr;
    }
  }
  return &dubinsNodes[i - 1];
}
Node3D* Algorithm::ArcShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace){
    Point_2 p1(start.getX(), start.getY()), p2(goal.getX(), goal.getY());
    double theta1 = start.getT(), theta2 = goal.getT();
    Vector_2 dir1(std::cos(theta1), std::sin(theta1)), dir2(std::cos(theta2), std::sin(theta2));
    Line_2 tangent1(p1, p1 + dir1), tangent2(p2, p2 + dir2);
    Line_2 perp1 = tangent1.perpendicular(p1), perp2 = tangent2.perpendicular(p2);
    CGAL::Object result = CGAL::intersection(perp1, perp2);
    Point_2 center;
    if (const Point_2 *ipoint = CGAL::object_cast<Point_2>(&result)) {
        center = *ipoint;
    } else {
        return nullptr;
    }
    Kernel::FT square_radius = CGAL::squared_distance(center, p1);
    double radius = std::sqrt(CGAL::to_double(square_radius));
    Vector_2 radiusDirStart = p1 - center;
    Vector_2 radiusDirEnd = p2 - center;
    double startAngle = std::atan2(CGAL::to_double(radiusDirStart.y()), CGAL::to_double(radiusDirStart.x()));
    double endAngle = std::atan2(CGAL::to_double(radiusDirEnd.y()), CGAL::to_double(radiusDirEnd.x()));
    double angelFromRadiusToVehicle = theta1 - Helper::normalizeHeadingRad(startAngle);
    if(angelFromRadiusToVehicle > M_PI){
        angelFromRadiusToVehicle -= 2 * M_PI;
    } else if(angelFromRadiusToVehicle < -M_PI){
        angelFromRadiusToVehicle += 2 * M_PI;
    }
    double deltaAngle = endAngle - startAngle;
    if(deltaAngle > M_PI) {
        deltaAngle -= 2 * M_PI;
    } else if(deltaAngle < -M_PI) {
        deltaAngle += 2 * M_PI;
    }
    double eachAngle = Node3D::dx[0] / radius;
    int numPoints = static_cast<int>(std::abs(deltaAngle) / eachAngle) + 2; 
    Node3D* arcNodes = new Node3D[numPoints];
    double mul = deltaAngle > 0 ? 1.0 : -1.0;
    int i = 0;
    for (double angle = 0; angle <= std::abs(deltaAngle); angle += eachAngle) {
        double currentAngle = mul * angle+startAngle;
        double x = CGAL::to_double(center.x()) + radius * std::cos(currentAngle);
        double y = CGAL::to_double(center.y()) + radius * std::sin(currentAngle);
        arcNodes[i].setX(x);
        arcNodes[i].setY(y);
        arcNodes[i].setT(Helper::normalizeHeadingRad(currentAngle+angelFromRadiusToVehicle));
        if (i > 0) {
            arcNodes[i].setPred(&arcNodes[i - 1]);
        } else {
            arcNodes[i].setPred(&start);
        }
        if (configurationSpace.isTraversable(&arcNodes[i])) {
            i++;
        } else {
            delete [] arcNodes; 
            return nullptr;
        }
    }
    arcNodes[i] = goal; 
    arcNodes[i].setPred(&arcNodes[i - 1]);
    return &arcNodes[i];
}