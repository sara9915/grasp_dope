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

    /* Calling planning action server */
    // actionlib::SimpleActionClient<grasp_dope
    actionlib::SimpleActionClient<grasp_dope::goal_pose_plan_Action> ac("planning_action", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); // will wait for infinite time
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
    ac.sendGoalAndWait(goal);
    ROS_INFO_STREAM(ac.getResult()->success);

    // ros::spin();
    return 0;
}