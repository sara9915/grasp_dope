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

    /* Pick pose definition */
    goal.goal_pose_pick.pose = grasp_pose->pose;
    goal.goal_pose_pick.pose.position.z = goal.goal_pose_pick.pose.position.z- 0.01;
    goal.goal_pose_pick.pose.position.y = goal.goal_pose_pick.pose.position.y;// - 0.04;
    goal.goal_pose_pick.pose.position.x = goal.goal_pose_pick.pose.position.x;// + 0.01;

    /* Place pose definition */
    goal.goal_pose_place.pose.position.y = -0.28;
    goal.goal_pose_place.pose.position.x = 0.50;
    goal.goal_pose_place.pose.position.z = 0.05-0.08;

    // Eigen::Matrix3d place_rotation;
    // place_rotation << -1,0,0,
    //                     0,1,0,
    //                     0,0,-1;

    // Eigen::Quaterniond place_quaternion(place_rotation);
    // goal.goal_pose_place.pose.orientation.w = place_quaternion.w();
    // goal.goal_pose_place.pose.orientation.x = place_quaternion.x();
    // goal.goal_pose_place.pose.orientation.y = place_quaternion.y();
    // goal.goal_pose_place.pose.orientation.z = place_quaternion.z();

    /* Planning and execute to pre_grasp_pose*/
    ac.sendGoalAndWait(goal);
    ROS_INFO_STREAM(ac.getResult()->success);


    //ros::spin();
    return 0;
}