#include "ros/ros.h"
#include <std_srvs/Trigger.h>
#include "grasp_dope/desired_grasp_pose_activate.h"
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include "grasp_dope/goal_pose_plan_Action.h"
#include <eigen3/Eigen/Geometry> 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_action");
    ros::NodeHandle nh;

    /* Calling building_scene service */
    {
        ROS_INFO_STREAM("--- Activating building_scene service ---");
        ros::service::waitForService("/build_scene");
        std_srvs::Trigger::Request req;
        req = {};
        std_srvs::Trigger::Response res;
        if (!ros::service::call<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("/build_scene", req, res))
        {
            ROS_INFO_STREAM("Error Creating scene...");
            return -1;
        }
    }


    /*Activating get_grasp_pose service*/
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

    auto grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/grasp_pose"); 
    ROS_INFO_STREAM(grasp_pose->pose);

    /* Deactivating get_grasp_pose service*/
    ROS_INFO_STREAM("--- Deactivating get_grasp_pose service ---");
    req.activate = false;
    if (!ros::service::call<grasp_dope::desired_grasp_pose_activate::Request, grasp_dope::desired_grasp_pose_activate::Response>("/get_grasp_pose_service", req, res))
    {
        ROS_INFO_STREAM("Error deactivating service get_grasp_pose...");
        return -1;
    }


    /* Calling planning action server */
    //actionlib::SimpleActionClient<grasp_dope
    actionlib::SimpleActionClient<grasp_dope::goal_pose_plan_Action> ac("planning_action", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); // will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    grasp_dope::goal_pose_plan_Goal goal;
    goal.goal_pose = grasp_pose->pose;

    // wait for the action to return
    ac.sendGoalAndWait(goal);
    ROS_INFO_STREAM("Finish with state: " << ac.getState().state_);

    ros::spin();
    return 0;
}