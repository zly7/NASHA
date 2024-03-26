


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>


nav_msgs::OccupancyGridPtr grid;


void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  std::cout << "Creating transform for map..." << std::endl;
  grid = map;
}

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;

  
  ros::Subscriber sub_map = n.subscribe("/occ_map", 1, setMap);
  tf::Pose tfPose;


  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;

  while (ros::ok()) {
    
    if (grid != nullptr) {
      tf::poseMsgToTF(grid->info.origin, tfPose);
    }

    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tfPose.getOrigin()),
        ros::Time::now(), "odom", "map"));

    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(), "map", "path"));
    ros::spinOnce();
    r.sleep();
  }
}
