







#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"
#include <yaml-cpp/yaml.h>
#include <regex>
#include "constants.h"
#include <dirent.h> 
#include <thread>
#include <chrono>
#include <atomic>

std::atomic<bool> keep_running(true);

void flushEverySecond(std::ofstream& out) {
    while (keep_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        out.flush();
    }
}




template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}





int main(int argc, char** argv) {
  message("resolution: ", HybridAStar::Constants::each_meter_to_how_many_pixel);

  if (HybridAStar::Constants::manual) {
    message("mode: ", "manual");
  } else {
    message("mode: ", "auto");
  }

  ros::init(argc, argv, "a_star");


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
  std::string baseDir = "/home/zly/plannerAll/catkin_path_planner/finalTime/";
  std::string dirPath = baseDir + (HybridAStar::Constants::algorithm == "contour_hybrid_astar" ? "ENHA" :
                                  HybridAStar::Constants::algorithm == "hybrid_astar" ? "HA" : 
                                  HybridAStar::Constants::algorithm == "rrt" ? "RRT" : "EHHA");
  std::string filePattern = "TPCAP_" + unique_number + "_";
  struct dirent* entry;
  int maxIndex = -1;
  DIR* dir = opendir(dirPath.c_str());
  if (dir == NULL) {
      std::cerr << "Failed to open directory" << std::endl;
      return -1; 
  }
  while ((entry = readdir(dir)) != NULL) {
      std::string filename(entry->d_name);
      if (filename.find(filePattern) != std::string::npos) { 
          size_t pos = filename.rfind('_'); 
          if (pos != std::string::npos) {
              int index = std::stoi(filename.substr(pos + 1));
              if (index > maxIndex) {
                  maxIndex = index;
              }
          }
      }
  }
  closedir(dir);
  std::ofstream out;
  std::string outPath = dirPath + "/" + filePattern + std::to_string(maxIndex + 1) + ".txt";
  out.open(outPath, std::ios::out);
  std::thread flush_thread(flushEverySecond, std::ref(out));
  std::streambuf *coutbuf = std::cout.rdbuf();
  std::cout.rdbuf(out.rdbuf());

  HybridAStar::Planner hy;
  hy.plan(); 

  ros::spin();
  
  keep_running = false;
  flush_thread.join(); 
  std::cout.rdbuf(coutbuf);
  out.close();
  return 0;
}
