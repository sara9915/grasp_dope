#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include "grasp_dope/depth_optimizer.h"
#include <eigen3/Eigen/Geometry>
#include <complex>

#define WIDTH 640
#define HEIGHT 480
#define DEPTH_SCALE 0.001
#define SAMPLE_AVERAGE 30

int sample_pose = 0;
int sample_depth = 0;

geometry_msgs::PoseStamped estimated_pose_msgs[SAMPLE_AVERAGE];
sensor_msgs::Image depth_msgs[SAMPLE_AVERAGE];

geometry_msgs::PoseStamped estimated_pose_msg;
float depth_matrix[HEIGHT * WIDTH];
cv::Mat depth_cv[SAMPLE_AVERAGE];

bool read_depth = false;
bool read_pose = false;

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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ref");
    ros::NodeHandle nh;
    ros::Subscriber aligned_depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &depth_callback);
    ros::Subscriber estimated_pose_sub = nh.subscribe("/dope/pose_obj", 1, &estimate_pose_callback);
    ros::Publisher optimized_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("optimized_pose", 1);

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
    
    geometry_msgs::PoseStamped optimized_pose_msg;

    if (client.call(srv))
    {
        if(srv.response.success)
        {
            optimized_pose_msg = srv.response.refined_pose;
        }
        else 
        {
            ROS_INFO_STREAM("Optimization failed!");
            optimized_pose_msg = srv.response.refined_pose;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    while (ros::ok())
    {
        optimized_pose_pub.publish(optimized_pose_msg);
    }

    return 0;
}