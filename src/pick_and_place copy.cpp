#include "ros/ros.h"
#include <std_srvs/Trigger.h>
#include "grasp_dope/desired_grasp_pose_activate.h"
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include "grasp_dope/goal_pose_plan_Action.h"
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <visp_tracking/tracking_mode_Action.h>
#include <tf/transform_listener.h>

void modify_wrl(const std::string &model_color, const std::string &extension_file, const std::string &folder_file, const std::string &replace, const std::string &replace_with)
{
    std::cout << "Opening: " + folder_file << model_color << extension_file << std::endl;
    std::ifstream input_file(folder_file + model_color + "_original" + extension_file);

    if (input_file.is_open())
    {
        std::string line;
        std::vector<std::string> lines;

        while (std::getline(input_file, line))
        {
            // std::cout << line << std::endl;

            std::string::size_type pos = 0;

            while ((pos = line.find(replace, pos)) != std::string::npos)
            {
                line.replace(pos, line.size(), replace_with);
                pos += replace_with.size();
            }

            lines.push_back(line);
        }

        input_file.close();
        std::ofstream output_file(folder_file + model_color + extension_file);
        if (output_file.is_open())
        {
            for (const auto &i : lines)
            {
                output_file << i << std::endl;
            }
        }
        output_file.close();
    }
    else
    {
        std::cout << "Error opening file" << folder_file << model_color << extension_file;
    }
}

geometry_msgs::PoseStamped tracking_init_pose(const geometry_msgs::PoseStamped &obj_pose)
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
    /* Object pose detected with best score */
    Eigen::Vector3d translation_OB(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z);
    Eigen::Quaterniond orientation_OB(obj_pose.pose.orientation.w, obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z);
    Eigen::Isometry3d T_OB(orientation_OB);
    T_OB.translation() = translation_OB;

    Eigen::Quaterniond orientation_CB(transform_CB.getRotation());
    Eigen::Vector3d translation_CB(transform_CB.getOrigin());
    Eigen::Isometry3d T_CB(orientation_CB);
    T_CB.translation() = translation_CB;

    /* Calculate frame object-base*/
    Eigen::Isometry3d T_OC;
    T_OC = T_CB.inverse() * T_OB;

    geometry_msgs::PoseStamped tracker_init_pose;
    Eigen::Quaterniond orientation_init(T_OC.rotation());
    tracker_init_pose.header = obj_pose.header;
    tracker_init_pose.header.frame_id = "camera_color_optical_frame";
    tracker_init_pose.pose.position.x = T_OC.translation().x();
    tracker_init_pose.pose.position.y = T_OC.translation().y();
    tracker_init_pose.pose.position.z = T_OC.translation().z();

    tracker_init_pose.pose.orientation.w = orientation_init.w();
    tracker_init_pose.pose.orientation.x = orientation_init.x();
    tracker_init_pose.pose.orientation.y = orientation_init.y();
    tracker_init_pose.pose.orientation.z = orientation_init.z();
    ROS_INFO_STREAM(tracker_init_pose);
    return tracker_init_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_action");
    ros::NodeHandle nh;

    /* Calling building_scene service */
    // {
    //     ROS_INFO_STREAM("--- Activating building_scene service ---");
    //     ros::service::waitForService("/build_scene");
    //     std_srvs::Trigger::Request req;
    //     req = {};
    //     std_srvs::Trigger::Response res;
    //     if (!ros::service::call<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("/build_scene", req, res))
    //     {
    //         ROS_INFO_STREAM("Error Creating scene...");
    //         return -1;
    //     }
    // }

    /*Activating get_grasp_pose service*/
    std::cout << "Press Enter to Start";
    std::cin.ignore();
    ROS_INFO_STREAM("--- Activating get_grasp_pose service ---");
    ros::service::waitForService("/get_grasp_pose_service");
    grasp_dope::desired_grasp_pose_activate::Request req;
    req.activate = true;
    grasp_dope::desired_grasp_pose_activate::Response res;
    if (!ros::service::call<grasp_dope::desired_grasp_pose_activate::Request, grasp_dope::desired_grasp_pose_activate::Response>("/get_grasp_pose_service", req, res))
    {
        ROS_INFO_STREAM("Error activating service get_grasp_pose...");
        return -1;
    }
    geometry_msgs::PoseStamped grasp_pose = res.refined_pose;

    /* Calling tracking action server */
    actionlib::SimpleActionClient<visp_tracking::tracking_mode_Action> ac_tracking("tracker_as", true);

    ROS_INFO("Waiting for tracker action server to start.");
    ac_tracking.waitForServer(); // will wait for infinite time
    ROS_INFO("Tracker action server started, sending goal.");
    visp_tracking::tracking_mode_Goal goal_tracker;

    /* First scale cad model in wrl file */
    std::string model_color = "banana2_centered_46_88_vrt_fc";
    std::string extension_file = ".wrl";
    std::string folder_file = "/home/workstation/dope_ros_ws/src/visp_tracking/src/generic-rgbd/model/banana_6/source/banana/46_88/";
    
    std::cout << "Replacing string..." << std::endl;
    std::string replace = "  scale 1 1 1";
    std::string replace_with = "  scale 0.01 0.01 0.01"; //"  scale " + std::to_string(res.scale_obj) + " " + std::to_string(res.scale_obj) + " " + std::to_string(res.scale_obj);
    std::cout << replace_with << std::endl;
    modify_wrl(model_color, extension_file, folder_file, replace, replace_with);
    std::cout << "Replaced string..." << std::endl;

    goal_tracker.use_depth = true;
    goal_tracker.use_edges = false;
    goal_tracker.use_ktl = false;
    goal_tracker.path_wrl = folder_file + model_color + extension_file; //"/home/workstation/dope_ros_ws/src/visp_tracking/src/generic-rgbd/model/banana_6/source/banana/46_88/banana2_centered_46_88_vrt_fc.wrl";
    goal_tracker.initial_pose = tracking_init_pose(res.refined_pose);
    /* Tracker activate */
    ac_tracking.sendGoal(goal_tracker);
    // ROS_INFO_STREAM(ac_tracking.getResult()->success);

    /* Calling planning action server */
    // actionlib::SimpleActionClient<grasp_dope
    actionlib::SimpleActionClient<grasp_dope::goal_pose_plan_Action> ac_planning("planning_action", true);

    ROS_INFO("Waiting for action server to start.");
    ac_planning.waitForServer(); // will wait for infinite time
    ROS_INFO("Action server started, sending goal.");
    grasp_dope::goal_pose_plan_Goal goal;

    /* Pick pose definition */
    goal.goal_pose_pick.pose = grasp_pose.pose;
    goal.goal_pose_pick.pose.position.z = goal.goal_pose_pick.pose.position.z;
    goal.goal_pose_pick.pose.position.y = goal.goal_pose_pick.pose.position.y;
    goal.goal_pose_pick.pose.position.x = goal.goal_pose_pick.pose.position.x;

    /* Place pose definition */
    goal.goal_pose_place.pose.position.y = -0.28;
    goal.goal_pose_place.pose.position.x = 0.50;
    goal.goal_pose_place.pose.position.z = 0.05 - 0.08;

    goal.scaled_cuboid_dimensions = res.scaled_cuboid_dimensions;
    goal.scale_obj = res.scale_obj;

    /* Planning and execute to pre_grasp_pose*/
    ac_planning.sendGoalAndWait(goal);
    ROS_INFO_STREAM(ac_planning.getResult()->success);

    // ros::spin();
    return 0;
}