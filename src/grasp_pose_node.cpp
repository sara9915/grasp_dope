/**
    This node:
        -> read poses detected by inference_dope_node 
        -> elaborate poses to grasp
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grasp_dope/desired_grasp_pose_activate.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Geometry> 

bool state = false;

bool get_grasp_pose(grasp_dope::desired_grasp_pose_activate::Request  &req,
         grasp_dope::desired_grasp_pose_activate::Response &res)
{
  res.state = req.activate;
  state = res.state;
  if(res.state == true) ROS_INFO_STREAM("Service get_grasp_pose: ACTIVATED");
  else ROS_INFO_STREAM("Service get_grasp_pose: DEACTIVATED");
  return state;
}


void calculate_pose(const geometry_msgs::PoseStamped::ConstPtr& obj_pose)
{
  ROS_INFO_STREAM("Reading object poses...");
  /* Object pose detected with best score */
  // ROS_INFO_STREAM(obj_pose->pose);
  Eigen::Vector3d translation(obj_pose->pose.position.x, obj_pose->pose.position.y, obj_pose->pose.position.z);
  Eigen::Quaterniond orientation(obj_pose->pose.orientation.w,obj_pose->pose.orientation.x,obj_pose->pose.orientation.y,obj_pose->pose.orientation.z);
  Eigen::Isometry3d T_OC(orientation);
  T_OC.translation() = translation;

  ROS_INFO_STREAM("T_OC translation: " << "\n" << T_OC.translation());
  ROS_INFO_STREAM("T_OC orientation: " << "\n" << T_OC.rotation());

  Eigen::Quaterniond unit_quat(1,0,0,0);
  Eigen::Isometry3d T_CB(unit_quat);
  Eigen::Isometry3d T_OB;
  T_OB = T_CB*T_OC;

  ROS_INFO_STREAM("T_OB translation: " << "\n" << T_OB.translation());
  ROS_INFO_STREAM("T_OB orientation: " << "\n" << T_OB.rotation());
  

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_pose_node");

  ros::AsyncSpinner spinner(4); 
  spinner.start();
  
  ros::NodeHandle n;
  ros::Subscriber sub; 
  

  /* This service is used to activate this node: it must be activated only if manipulator is stopped*/
  ros::ServiceServer service = n.advertiseService("get_grasp_pose_service", get_grasp_pose);


  /* Reading detected object pose in camera frame */
  while(ros::ok())
  {
    if(state==true && sub == nullptr) 
    {
      ROS_INFO_STREAM("Creating subscriber...");
      sub = n.subscribe("/dope/pose_obj", 1, calculate_pose);
    }
    if(state==false && sub != nullptr) 
    {
      ROS_INFO_STREAM("Destroying subscriber...");
      sub.shutdown();
    }
  }

  ros::waitForShutdown();

  return 0;
}