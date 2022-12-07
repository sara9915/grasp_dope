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
#include <tf/transform_listener.h>

bool state = false;
ros::Publisher grasp_pose_pub;

bool get_grasp_pose(grasp_dope::desired_grasp_pose_activate::Request  &req,
         grasp_dope::desired_grasp_pose_activate::Response &res)
{
  res.state = req.activate;
  state = res.state;
  if(res.state == true) ROS_INFO_STREAM("Service get_grasp_pose: ACTIVATED");
  else ROS_INFO_STREAM("Service get_grasp_pose: DEACTIVATED");
  return true;
}


void calculate_pose(const geometry_msgs::PoseStamped::ConstPtr& obj_pose)
{
  /* Reading frame camera_base */
  tf::TransformListener listener;
  tf::StampedTransform transform_CB;
  try
  {
    listener.waitForTransform("/base_link", "/camera_color_optical_frame", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/base_link", "/camera_color_optical_frame",  
                            ros::Time(0), transform_CB);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }


  ROS_INFO_STREAM("Reading object poses...");
  /* Object pose detected with best score */
  // ROS_INFO_STREAM(obj_pose->pose);
  Eigen::Vector3d translation_OC(obj_pose->pose.position.x, obj_pose->pose.position.y, obj_pose->pose.position.z);
  Eigen::Quaterniond orientation_OC(obj_pose->pose.orientation.w,obj_pose->pose.orientation.x,obj_pose->pose.orientation.y,obj_pose->pose.orientation.z);
  Eigen::Isometry3d T_OC(orientation_OC);
  T_OC.translation() = translation_OC;

  // ROS_INFO_STREAM("T_OC translation: " << "\n" << T_OC.translation());
  // ROS_INFO_STREAM("T_OC orientation: " << "\n" << T_OC.rotation());

  Eigen::Quaterniond orientation_CB(transform_CB.getRotation());
  Eigen::Vector3d translation_CB(transform_CB.getOrigin());
  Eigen::Isometry3d T_CB(orientation_CB);
  T_CB.translation() = translation_CB;

  // ROS_INFO_STREAM("T_CB translation: " << "\n" << T_CB.translation());
  // ROS_INFO_STREAM("T_CB orientation: " << "\n" << T_CB.rotation());

  /* Calculate frame object-base*/
  Eigen::Isometry3d T_OB;
  T_OB = T_CB*T_OC;

  // ROS_INFO_STREAM("T_OB translation: " << "\n" << T_OB.translation());
  // ROS_INFO_STREAM("T_OB orientation: " << "\n" << T_OB.rotation());


  /* Calculating grasp pose */
  Eigen::Matrix3d rotation_grasp_pose;
  rotation_grasp_pose << 0, -1, 0,
                        -1, 0, 0,
                        0, 0, -1;
  Eigen::Quaterniond orientation_grasp_pose(rotation_grasp_pose);
  double offset_z = 0.10;

  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = "base_link";
  grasp_pose_msg.pose.position.x = T_OB.translation().x();
  grasp_pose_msg.pose.position.y = T_OB.translation().y();
  grasp_pose_msg.pose.position.z = T_OB.translation().z() + offset_z;

  grasp_pose_msg.pose.orientation.w = orientation_grasp_pose.w();
  grasp_pose_msg.pose.orientation.x = orientation_grasp_pose.x();
  grasp_pose_msg.pose.orientation.y = orientation_grasp_pose.y();
  grasp_pose_msg.pose.orientation.z = orientation_grasp_pose.z();


  grasp_pose_pub.publish(grasp_pose_msg);
  ROS_INFO_STREAM("Grasp pose: ");
  ROS_INFO_STREAM(grasp_pose_msg);
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

  /* Publisher grasp_pose*/
  grasp_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/grasp_pose", 1);


  /* Reading detected object pose in camera frame */
  while(ros::ok())
  {
    if(state==true && sub == nullptr) 
    {
      /* Reading detected object pose */
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