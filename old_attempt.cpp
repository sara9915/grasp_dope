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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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
    // ROS_INFO_STREAM("Planning result: " << success);

    return my_plan;
}

auto planning_cartesian(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "end_effector_tool0");
    std::vector<geometry_msgs::Pose> waypoints;
    
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction != 1)
    {
        ROS_INFO_STREAM("Fraction value: " << fraction);
        success = 0;
    }
    else
        success = 1;

    return trajectory;
}

auto get_tool_pose(const geometry_msgs::Pose &pose)
{
    Eigen::Isometry3d T_TE; // omogeneous transfrom from tool0 to end-effector_tool0
    T_TE.translation().x() = 0.0;
    T_TE.translation().y() = 0.0;
    T_TE.translation().z() = -0.21;

    Eigen::Quaterniond q_EB(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Isometry3d T_EB(q_EB);
    T_EB.translation().x() = pose.position.x;
    T_EB.translation().y() = pose.position.y;
    T_EB.translation().z() = pose.position.z;

    Eigen::Isometry3d T_TB(T_EB);
    Eigen::Isometry3d temp(T_EB * T_TE);
    T_TB.translation() = temp.translation();

    geometry_msgs::Pose start_state_pose;
    Eigen::Quaterniond start_state_quat(T_TB.rotation());
    start_state_pose.orientation.w = start_state_quat.w();
    start_state_pose.orientation.x = start_state_quat.x();
    start_state_pose.orientation.y = start_state_quat.y();
    start_state_pose.orientation.z = start_state_quat.z();

    start_state_pose.position.x = T_TB.translation().x();
    start_state_pose.position.y = T_TB.translation().y();
    start_state_pose.position.z = T_TB.translation().z();

    return start_state_pose;
}

void attach_obj(moveit::planning_interface::MoveGroupInterface &move_group_interface, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, geometry_msgs::Pose pose_obj)
{
    moveit_msgs::CollisionObject object_to_attach;
    shape_msgs::SolidPrimitive primitive;
    object_to_attach.id = "obj";

    shape_msgs::SolidPrimitive box_primitive;
    box_primitive.type = primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[primitive.BOX_X] = 0.08; //box_size->detections[2].bbox.size.x;
    box_primitive.dimensions[primitive.BOX_Y] = 0.08; //box_size->detections[2].bbox.size.y;
    box_primitive.dimensions[primitive.BOX_Z] = 0.08; //box_size->detections[2].bbox.size.z;

    // La posizione del box è definita rispetto alla terna world
    object_to_attach.header.frame_id = "base_link";

    // First, we add the object to the world (without using a vector)
    object_to_attach.primitives.push_back(box_primitive);
    object_to_attach.primitive_poses.push_back(pose_obj);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    move_group_interface.attachObject(object_to_attach.id, "end_effector_tool0");
}

bool executeCB(const grasp_dope::goal_pose_plan_GoalConstPtr &goal, actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> *as, ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state)
{
    // create messages that are used to published feedback/result
    grasp_dope::goal_pose_plan_Feedback feedback;
    grasp_dope::goal_pose_plan_Result result;
    // bool success = true;

    geometry_msgs::Pose pre_grasp_pose;
    geometry_msgs::Pose grasp_pose;
    geometry_msgs::Pose post_grasp_pose;
    geometry_msgs::Pose place_pose;

    moveit::core::RobotState start_state(*move_group_interface->getCurrentState());

    moveit::planning_interface::MoveGroupInterface::Plan plan_pre_grasp;
    moveit_msgs::RobotTrajectory plan_pick;
    moveit_msgs::RobotTrajectory plan_post_grasp;
    moveit::planning_interface::MoveGroupInterface::Plan plan_place;

    std::vector<geometry_msgs::Pose> pre_grasp_attemp;

    Eigen::Matrix3d rotation_start;
    Eigen::Matrix3d rotation_alpha;
    Eigen::Matrix3d rotation_theta;

    double alpha = 0.0; // rotazione attorno all'oggetto
    double theta = 0.0; // inclinazione rispetto all'oggetto
    double offset = 0.13;
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;

    rotation_start << 0, -1, 0,
                    -1, 0, 0,
                    0, 0, -1;
    int rotation_attempt = 3;
    int inclination_attempt = 5;

    for (int i = -inclination_attempt; i < inclination_attempt; i++)
    {
        theta = (i * M_PI / inclination_attempt);

        /* Rotazione attorno all'asse y */
        rotation_theta << cos(theta), 0, sin(theta),
                           0, 1, 0,
                        -sin(theta), 0, cos(theta);

        for(int k = 0; k < rotation_attempt; k++)
        {
            alpha = (k * 2 * M_PI / rotation_attempt);
            pre_grasp_attemp.at(k).position.x = goal->goal_pose_pick.pose.position.x + offset*cos(theta);
            pre_grasp_attemp.at(k).position.y = goal->goal_pose_pick.pose.position.y + offset*sin(theta)*sin(alpha);
            pre_grasp_attemp.at(k).position.z = goal->goal_pose_pick.pose.position.z + offset*sin(theta)*cos(alpha);
        
            /* Matrice di rotazione attorno all'asse z */
            rotation_alpha << cos(alpha), -sin(alpha), 0,
                                sin(alpha), cos(alpha), 0,
                                0, 0, 1;

            Eigen::Quaterniond q_(rotation_start * rotation_alpha * rotation_theta);
            pre_grasp_attemp.at(k).orientation.w = q_.w();
            pre_grasp_attemp.at(k).orientation.x = q_.x();
            pre_grasp_attemp.at(k).orientation.y = q_.y();
            pre_grasp_attemp.at(k).orientation.z = q_.z();
        }
    }

    
    int attempt = 0;

    while (!success_planning_pp && attempt < pre_grasp_attemp.size())
    {
        ROS_INFO_STREAM(attempt + 1 << " ATTEMPT");

        pre_grasp_pose = goal->goal_pose_pick.pose;
        pre_grasp_pose.position.z = pre_grasp_pose.position.z + offset_z;
        pre_grasp_pose.orientation = orientation_attempt.at(attempt);

        grasp_pose = goal->goal_pose_pick.pose;
        grasp_pose.orientation = orientation_attempt.at(attempt);

        post_grasp_pose = pre_grasp_pose;

        place_pose = goal->goal_pose_place.pose;
        place_pose.orientation = orientation_attempt.at(attempt);

        ROS_INFO_STREAM("--- Pre-grasp pose --- "
                        << "\n"
                        << pre_grasp_pose);

        if (as->isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            // set the action state to preempted
            as->setPreempted();
            success = false;
            return success;
        }

        /* Planning to pre-grasp pose */
        moveit::core::RobotState start_state(*move_group_interface->getCurrentState());
        move_group_interface->setStartState(start_state);

        plan_pre_grasp = planning_joint(pre_grasp_pose, *move_group_interface);
        ROS_INFO_STREAM("Result planning pre-grasp pose: " << success);

        if (success)
        {
            std::cout << "Press Enter to Continue";
            std::cin.ignore();

            ROS_INFO_STREAM("--- Grasp pose ---"
                            << "\n"
                            << grasp_pose);

            /* Planning to grasp pose */
            start_state.setFromIK(joint_model_group, get_tool_pose(pre_grasp_pose)); 
            move_group_interface->setStartState(start_state);

            plan_pick = planning_cartesian(get_tool_pose(grasp_pose), *move_group_interface);
 
            ROS_INFO_STREAM("Result planning grasp pose: " << success);
            if (success)
            {
                std::cout << "Press Enter to Continue";
                std::cin.ignore();
                /* Planning to post grasp pose */
                ROS_INFO_STREAM("--- Post grasp pose --- "
                                << "\n"
                                << post_grasp_pose);

                start_state.setFromIK(joint_model_group, get_tool_pose(grasp_pose));
                move_group_interface->setStartState(start_state);

                attach_obj(*move_group_interface, *planning_scene_interface, goal->goal_pose_pick.pose);
                plan_post_grasp = planning_cartesian(get_tool_pose(post_grasp_pose), *move_group_interface);
                ROS_INFO_STREAM("Result planning post grasp pose: " << success);

                if (success)
                {
                    std::cout << "Press Enter to Continue";
                    std::cin.ignore();
                    /* Planning to place pose */
                    ROS_INFO_STREAM("--- Place pose --- "
                                    << "\n"
                                    << place_pose);
                    
                    start_state.setFromIK(joint_model_group, get_tool_pose(post_grasp_pose));
                    move_group_interface->setStartState(start_state);

                    plan_place = planning_joint(place_pose, *move_group_interface);
                    ROS_INFO_STREAM("Result planning place pose: " << success);

                    if (success)
                    {
                        std::cout << "Planning successfully completed. Press ENTER to continue...";
                        std::cin.ignore();
                        move_group_interface->detachObject("obj");
                        success_planning_pp = true;
                        attempt = 0;
                    }
                    else
                    {
                        ROS_INFO_STREAM("Try with other pre-grasp pose...");
                        attempt = attempt + 1;
                    }
                }
                else
                {
                    ROS_INFO_STREAM("Try with other pre-grasp pose...");
                    attempt = attempt + 1;
                }
            }
            else
            {
                move_group_interface->detachObject("obj");
                ROS_INFO_STREAM("Try with other pre-grasp pose...");
                attempt = attempt + 1;
            }
        }
        else
        {
            ROS_INFO_STREAM("Try with other pre-grasp pose...");
            attempt = attempt + 1;
        }
    }

    result.success = success;
    as->setSucceeded(result);

    if (success_planning_pp)
    {
        ROS_INFO_STREAM("Executing trajectory pre_grasp...");
        move_group_interface->execute(plan_pre_grasp);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("Executing trajectory pick...");
        move_group_interface->execute(plan_pick);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        attach_obj(*move_group_interface, *planning_scene_interface, goal->goal_pose_pick.pose);

        ROS_INFO_STREAM("Executing trajectory post_grasp...");
        move_group_interface->execute(plan_post_grasp);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("Executing trajectory place...");
        move_group_interface->execute(plan_place);

        move_group_interface->detachObject("obj");
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

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ros::Duration timeout_box_size(1);
    /* Subscriber to read size of detected obj */
    box_size = ros::topic::waitForMessage<vision_msgs::Detection3DArray>("/dope/detected_objects", timeout_box_size);

    /* Creazione del ros action */
    actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> as(nh, "planning_action", boost::bind(&executeCB, _1, &as, &nh, &move_group_interface, &planning_scene_interface, joint_model_group, kinematic_state), false); // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    as.start();

    ros::waitForShutdown();

    return 0;
}