#include "ros/ros.h"

#include "grasp_dope/goal_pose_plan_Action.h"
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


bool planning_executePoseTarget(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "gripper");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    ROS_INFO_STREAM("Target Pose: \n" << target_pose);

    bool success;
    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setPlannerId("RRTstarkConfigDefault");
    
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Planning result: " << success);

    // if (success)
    // {   
        
    //     move_group_interface.execute(my_plan);
    //     ROS_INFO_STREAM("Executing trajectory...");
    // }
    return success;
}

bool executeCB(const grasp_dope::goal_pose_plan_GoalConstPtr &goal, actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> *as, ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface)
{
    // create messages that are used to published feedback/result
    grasp_dope::goal_pose_plan_Feedback feedback;
    grasp_dope::goal_pose_plan_Result result;
    bool success = true;
    if (as->isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("Preempted");
        // set the action state to preempted
        as->setPreempted();
        success = false;
        return success;
    }
    /* Planning to goal pose */
    if (planning_executePoseTarget(goal->goal_pose, *move_group_interface))
    {
        result.success = success;
        ROS_INFO("Succeeded");
        // set the action state to succeeded
        as->setSucceeded(result);
    }
    else 
    {
        result.success = false;
        ROS_INFO("Not Succeeded");
        // set the action state to succeeded
        as->setSucceeded(result);
    }

    return success;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_server");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    /* Parametri per la costruzione della scena */
    static const std::string PLANNING_GROUP = "yaskawa_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    /* Creazione del ros action */
    actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> as(nh, "planning_action", boost::bind(&executeCB, _1, &as, &nh, &move_group_interface, &planning_scene_interface), false); // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    as.start();

    ros::waitForShutdown();

    return 0;
}