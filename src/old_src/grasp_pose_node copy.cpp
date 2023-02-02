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
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "grasp_dope/depth_optimizer.h"

#define WIDTH 640
#define HEIGHT 480
#define DEPTH_SCALE 0.001
#define SAMPLE_AVERAGE 30

int sample_pose = 0;
int sample_depth = 0;

geometry_msgs::PoseStamped estimated_pose_msgs[SAMPLE_AVERAGE];
sensor_msgs::Image depth_msgs[SAMPLE_AVERAGE];

geometry_msgs::PoseStamped estimated_pose_msg;
geometry_msgs::PoseStamped grasp_pose_msg;
float depth_matrix[HEIGHT * WIDTH];
cv::Mat depth_cv[SAMPLE_AVERAGE];

bool read_depth = false;
bool read_pose = false;

bool state = false;
ros::Publisher grasp_pose_pub;

bool get_grasp_pose(grasp_dope::desired_grasp_pose_activate::Request &req,
                    grasp_dope::desired_grasp_pose_activate::Response &res)
{
  ROS_INFO_STREAM("Callback get_grasp_pose_service");
  if (!req.activate && state == true)
  {
    ROS_INFO_STREAM("Cannot deactivate service, please wait for termination...");
    res.state = state;
  }
  else
  {
    res.state = req.activate;
    state = res.state;
    ROS_INFO_STREAM("Service state: " << state);
  }
  return true;
}

void estimate_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (sample_pose < SAMPLE_AVERAGE)
  {
    estimated_pose_msgs[sample_pose].pose = msg->pose;
    sample_pose = sample_pose + 1;
  }
  if (sample_pose == SAMPLE_AVERAGE)
  {
    read_pose = true;
  }
}

void depth_callback(const sensor_msgs::ImageConstPtr &msg)
{
  if (sample_pose > 0 && sample_depth <= SAMPLE_AVERAGE)
  {
    depth_msgs[sample_depth].data = msg->data;
    depth_msgs[sample_depth].step = msg->step;
    depth_msgs[sample_depth].is_bigendian = msg->is_bigendian;
    depth_msgs[sample_depth].height = msg->height;
    depth_msgs[sample_depth].encoding = msg->encoding;
    depth_msgs[sample_depth].width = msg->width;
    sample_depth = sample_depth + 1;
  }

  if (sample_depth == SAMPLE_AVERAGE)
  {
    read_depth = true;
  }
}

void get_real_depth(sensor_msgs::Image &msg, cv::Mat &current_depth)
{
  current_depth = cv::Mat::zeros(cv::Size(WIDTH, HEIGHT), CV_16UC1);
  current_depth = cv_bridge::toCvCopy(msg, msg.encoding)->image;
}

void average_pose()
{
  for (int i = 0; i < SAMPLE_AVERAGE; i++)
  {
    estimated_pose_msg.pose.position.x = estimated_pose_msg.pose.position.x + estimated_pose_msgs[i].pose.position.x;
    estimated_pose_msg.pose.position.y = estimated_pose_msg.pose.position.y + estimated_pose_msgs[i].pose.position.y;
    estimated_pose_msg.pose.position.z = estimated_pose_msg.pose.position.z + estimated_pose_msgs[i].pose.position.z;

    estimated_pose_msg.pose.orientation.w = estimated_pose_msg.pose.orientation.w + estimated_pose_msgs[i].pose.orientation.w;
    estimated_pose_msg.pose.orientation.x = estimated_pose_msg.pose.orientation.x + estimated_pose_msgs[i].pose.orientation.x;
    estimated_pose_msg.pose.orientation.y = estimated_pose_msg.pose.orientation.y + estimated_pose_msgs[i].pose.orientation.y;
    estimated_pose_msg.pose.orientation.z = estimated_pose_msg.pose.orientation.z + estimated_pose_msgs[i].pose.orientation.z;
  }
  estimated_pose_msg.pose.position.x = estimated_pose_msg.pose.position.x / SAMPLE_AVERAGE;
  estimated_pose_msg.pose.position.y = estimated_pose_msg.pose.position.y / SAMPLE_AVERAGE;
  estimated_pose_msg.pose.position.z = estimated_pose_msg.pose.position.z / SAMPLE_AVERAGE;

  estimated_pose_msg.pose.orientation.w = estimated_pose_msg.pose.orientation.w / SAMPLE_AVERAGE;
  estimated_pose_msg.pose.orientation.x = estimated_pose_msg.pose.orientation.x / SAMPLE_AVERAGE;
  estimated_pose_msg.pose.orientation.y = estimated_pose_msg.pose.orientation.y / SAMPLE_AVERAGE;
  estimated_pose_msg.pose.orientation.z = estimated_pose_msg.pose.orientation.z / SAMPLE_AVERAGE;
}

void average_depth()
{

  for (int i = 0; i < SAMPLE_AVERAGE; i++)
  {
    get_real_depth(depth_msgs[i], depth_cv[i]);
  }

  for (int i = 0; i < HEIGHT; i++)
  {
    for (int j = 0; j < WIDTH; j++)
    {
      for (int k = 0; k < SAMPLE_AVERAGE; k++)
      {
        depth_matrix[i * WIDTH + j] = depth_matrix[i * WIDTH + j] + depth_cv[k].at<uint16_t>(i, j) * DEPTH_SCALE;
      }
      depth_matrix[i * WIDTH + j] = depth_matrix[i * WIDTH + j] / SAMPLE_AVERAGE;
    }
  }
}

void calculate_pose(const geometry_msgs::PoseStamped &obj_pose)
{
  /* Reading frame camera_base */
  tf::TransformListener listener;
  tf::StampedTransform transform_CB;
  try
  {
    listener.waitForTransform("/base_link", "/camera_color_optical_frame", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform("/base_link", "/camera_color_optical_frame",
                             ros::Time(0), transform_CB);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  ROS_INFO_STREAM("Reading object poses...");
  /* Object pose detected with best score */
  Eigen::Vector3d translation_OC(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z);
  Eigen::Quaterniond orientation_OC(obj_pose.pose.orientation.w, obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z);
  Eigen::Isometry3d T_OC(orientation_OC);
  T_OC.translation() = translation_OC;

  Eigen::Quaterniond orientation_CB(transform_CB.getRotation());
  Eigen::Vector3d translation_CB(transform_CB.getOrigin());
  Eigen::Isometry3d T_CB(orientation_CB);
  T_CB.translation() = translation_CB;

  /* Calculate frame object-base*/
  Eigen::Isometry3d T_OB;
  T_OB = T_CB * T_OC;

  /* Calculating grasp pose */
  Eigen::Quaterniond orientation_grasp_pose(T_OB.rotation());

  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = "base_link";
  grasp_pose_msg.pose.position.x = T_OB.translation().x();
  grasp_pose_msg.pose.position.y = T_OB.translation().y();
  grasp_pose_msg.pose.position.z = T_OB.translation().z();

  grasp_pose_msg.pose.orientation.w = orientation_grasp_pose.w();
  grasp_pose_msg.pose.orientation.x = orientation_grasp_pose.x();
  grasp_pose_msg.pose.orientation.y = orientation_grasp_pose.y();
  grasp_pose_msg.pose.orientation.z = orientation_grasp_pose.z();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_pose_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  ros::Subscriber aligned_depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &depth_callback);
  ros::Subscriber estimated_pose_sub = nh.subscribe("/dope/pose_obj", 1, &estimate_pose_callback);

  /* This service is used to activate this node: it must be activated only if manipulator is stopped*/
  ros::ServiceServer service = nh.advertiseService("/get_grasp_pose_service", get_grasp_pose);
  geometry_msgs::PoseStamped optimized_pose_msg;

  /* Publisher grasp_pose*/
  grasp_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/grasp_pose", 1);

  /* Reading detected object pose in camera frame */
  bool first_ = true;
  ROS_INFO_STREAM("First");
  while (ros::ok())
  {
    if (state == true)
    {
      first_ = false;
      while (!read_depth || !read_pose)
      {
        ros::spinOnce();
      }
      ROS_INFO_STREAM("Calculating average for poses...");
      average_pose();
      ROS_INFO_STREAM(estimated_pose_msg);

      ROS_INFO_STREAM("Calculating average for depth...");
      average_depth();

      ros::ServiceClient client = nh.serviceClient<grasp_dope::depth_optimizer>("depth_optimizer");
      grasp_dope::depth_optimizer srv;

      srv.request.depth_matrix.resize(WIDTH * HEIGHT);
      for (int i = 0; i < WIDTH * HEIGHT; i++)
        srv.request.depth_matrix[i] = depth_matrix[i];

      srv.request.estimated_pose = estimated_pose_msg;
      ros::service::waitForService("/depth_optimizer");

      if (client.call(srv))
      {
        if (srv.response.success)
        {
          optimized_pose_msg = srv.response.refined_pose;
        }
        else
        {
          ROS_INFO_STREAM("Optimization failed!");
          optimized_pose_msg = srv.response.refined_pose;
        }
        calculate_pose(optimized_pose_msg);
        state = false;
      }
      else
      {
        ROS_ERROR("Failed to call service");
        return 1;
      }

      read_depth = false;
      read_pose = false;
    }
    if (!first_)
    {
      grasp_pose_pub.publish(grasp_pose_msg);
      ROS_INFO_STREAM("Grasp pose: ");
      ROS_INFO_STREAM(grasp_pose_msg);
    }
  }

  ros::waitForShutdown();
  return 0;
}