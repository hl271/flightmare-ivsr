#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <flightros/fompl_planner/fompl_planner.hpp>

int main(int argc, char *argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  pub_traj_markers = nh.advertise<visualization_msgs::Marker>("/motion_planning/planned_path_markers", 1);
  // quad initialization
  motion_planning::quad_ptr_ = std::make_unique<Quadrotor>();
  // add camera
  motion_planning::rgb_camera_ = std::make_unique<RGBCamera>();

  std::cout << "Read PointCloud" << std::endl;
  motion_planning::verts = motion_planning::readPointCloud();

  std::cout << "Get Bounds" << std::endl;
  motion_planning::getBounds();

  std::cout << "Plan & stuff" << std::endl;
  while (!motion_planning::solution_found) {
    motion_planning::plan();
  }

  std::cout << "Execute" << std::endl;
  motion_planning::executePath();


  return 0;
}