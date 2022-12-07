#include "ros/ros.h"
#include <vision_msgs/Detection3DArray.h>

#include "grasp_dope/goal_pose_plan_Action.h"
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

bool success_planning_pp = false;
bool success;
vision_msgs::Detection3DArrayConstPtr box_size;

moveit::planning_interface::MoveGroupInterface::Plan planning_joint(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "end_effector_tool0");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setPlannerId("RRTstarkConfigDefault");
    
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Planning result: " << success);

    return my_plan;
}

auto planning_cartesian(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setEndEffectorLink("end_effector_tool0");
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if(fraction == -1) success = 0;
    else success = 1;

    return trajectory;
}

void attach_obj(moveit::planning_interface::MoveGroupInterface &move_group_interface, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, geometry_msgs::Pose pose_obj)
{
    moveit_msgs::CollisionObject object_to_attach;
    shape_msgs::SolidPrimitive primitive;
    object_to_attach.id = "obj";

    shape_msgs::SolidPrimitive box_primitive;
    box_primitive.type = primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[primitive.BOX_X] = box_size->detections[2].bbox.size.x;
    box_primitive.dimensions[primitive.BOX_Y] = box_size->detections[2].bbox.size.y;
    box_primitive.dimensions[primitive.BOX_Z] = box_size->detections[2].bbox.size.z;

    // La posizione del cilindro è definita rispetto alla terna world
    object_to_attach.header.frame_id = "base_link";
    
    // First, we add the object to the world (without using a vector)
    object_to_attach.primitives.push_back(box_primitive);
    object_to_attach.primitive_poses.push_back(pose_obj);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    move_group_interface.attachObject(object_to_attach.id, "end_effector_tool0");
}

bool executeCB(const grasp_dope::goal_pose_plan_GoalConstPtr &goal, actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> *as, ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface, const moveit::core::JointModelGroup *joint_model_group)
{
    // create messages that are used to published feedback/result
    grasp_dope::goal_pose_plan_Feedback feedback;
    grasp_dope::goal_pose_plan_Result result;
    //bool success = true;

    geometry_msgs::Pose pre_grasp_pose;
    geometry_msgs::Pose grasp_pose;
    geometry_msgs::Pose post_grasp_pose;

    moveit::core::RobotState start_state(*move_group_interface->getCurrentState());

    moveit::planning_interface::MoveGroupInterface::Plan plan_pre_grasp;
    moveit_msgs::RobotTrajectory plan_pick;
    moveit_msgs::RobotTrajectory plan_post_grasp;
    moveit::planning_interface::MoveGroupInterface::Plan plan_place;

    std::vector<geometry_msgs::Quaternion> orientation_attempt;

    Eigen::Matrix3d rotation_start1;
    Eigen::Matrix3d rotation_alpha1;
    geometry_msgs::Quaternion q1;

    double alpha = 0.0;
    rotation_start1 << 0, -1, 0,
                -1, 0, 0,
                0, 0, -1;
    int rotation_attempt = 1;

    for(int i=0; i<rotation_attempt; i++)
    {
        alpha = (i*2*M_PI/rotation_attempt);
        rotation_alpha1 << cos(alpha), -sin(alpha), 0,
                          sin(alpha), cos(alpha),0,
                          0, 0, 1;

        Eigen::Quaterniond q1_(rotation_start1*rotation_alpha1);
        q1.w = q1_.w();
        q1.x = q1_.x();
        q1.y = q1_.y();
        q1.z = q1_.z();

        orientation_attempt.push_back(q1);
    }
   

    // geometry_msgs::Quaternion q2;
    // orientation_attempt.push_back(q2);

    // geometry_msgs::Quaternion q3;
    // orientation_attempt.push_back(q3);

    // geometry_msgs::Quaternion q4;
    // orientation_attempt.push_back(q4);

    
    double offset_z = 0.0;
    double offset_y = 0.0;
    double offset_x = 0.0;
    int attempt = 0;


    while(!success_planning_pp && attempt < orientation_attempt.size())
    {
        moveit::core::RobotState start_state(*move_group_interface->getCurrentState());
        ROS_INFO_STREAM(attempt+1 << " ATTEMPT");
        offset_z = 0.20; //cm
        pre_grasp_pose = goal->goal_pose_pick.pose;
        pre_grasp_pose.position.z = pre_grasp_pose.position.z + offset_z;
        pre_grasp_pose.orientation = orientation_attempt.at(attempt);

        grasp_pose = goal->goal_pose_pick.pose;
        grasp_pose.orientation = orientation_attempt.at(attempt);

        post_grasp_pose = pre_grasp_pose;

        ROS_INFO_STREAM("Pre-grasp pose: " << pre_grasp_pose);

        if (as->isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            // set the action state to preempted
            as->setPreempted();
            success = false;
            return success;
        }

        /* Planning to pre-grasp pose */
        plan_pre_grasp = planning_joint(pre_grasp_pose, *move_group_interface);
        ROS_INFO_STREAM("Result planning pre-grasp pose: " << success);
        if(success)
        {
            /* Planning to grasp pose */
            ROS_INFO_STREAM("Grasp pose: " << grasp_pose);
            start_state.setFromIK(joint_model_group, pre_grasp_pose);
            move_group_interface->setStartState(start_state);
            plan_pick = planning_cartesian(goal->goal_pose_pick.pose, *move_group_interface);
            ROS_INFO_STREAM("Result planning grasp pose: " << success);
            if(success)
            {
                /* Planning to post grasp pose */
                ROS_INFO_STREAM("Post grasp pose: " << post_grasp_pose);
                start_state.setFromIK(joint_model_group, grasp_pose);
                move_group_interface->setStartState(start_state);
                //attach_obj(*move_group_interface, *planning_scene_interface, goal->goal_pose_pick.pose);
                plan_post_grasp = planning_cartesian(post_grasp_pose, *move_group_interface);
                ROS_INFO_STREAM("Result planning post grasp pose: " << success);

                if(success)
                {
                    // /* Planning to place pose */
                    // ROS_INFO_STREAM("Place pose: " << pre_grasp_pose);
                    // start_state.setFromIK(joint_model_group, post_grasp_pose);
                    // move_group_interface->setStartState(start_state);
                    // plan_place = planning_joint(goal->goal_pose_place.pose, *move_group_interface);
                    // ROS_INFO_STREAM("Result planning place pose: " << success);
                    
                    // if(success)
                    // {
                        //move_group_interface->detachObject(object_to_attach.id);
                        success_planning_pp = true;
                        attempt = 0;

                    //}
                    // else
                    // {
                    //     ROS_INFO_STREAM("Try with other pre-grasp pose...");
                    //     attempt = attempt +1;
                    // }
                }
                else
                {
                    ROS_INFO_STREAM("Try with other pre-grasp pose...");
                    attempt = attempt +1;
                }
            }
            else
            {
                ROS_INFO_STREAM("Try with other pre-grasp pose...");
                attempt = attempt +1;
            }
        }
        else
        {
            ROS_INFO_STREAM("Try with other pre-grasp pose...");
            attempt = attempt +1;
        }
        
    }

    result.success = success;
    as->setSucceeded(result);
    
    if (success_planning_pp)
    {      
        ROS_INFO_STREAM("Executing trajectory pre_grasp...");
        move_group_interface->execute(plan_pre_grasp);

        ROS_INFO_STREAM("Executing trajectory pick...");
        move_group_interface->execute(plan_pick);

        ROS_INFO_STREAM("Executing trajectory post_grasp...");
        move_group_interface->execute(plan_post_grasp);

        // ROS_INFO_STREAM("Executing trajectory place...");
        // move_group_interface->execute(plan_place);
    }
    
    return true;
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

    
    /* Subscriber to read size of detected obj */
    box_size = ros::topic::waitForMessage<vision_msgs::Detection3DArray>("/dope/detected_objects");


    /* Creazione del ros action */
    actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> as(nh, "planning_action", boost::bind(&executeCB, _1, &as, &nh, &move_group_interface, &planning_scene_interface, joint_model_group), false); // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    as.start();

    ros::waitForShutdown();

    return 0;
}