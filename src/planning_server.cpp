#include <math.h> /* pow */
#include "ros/ros.h"
#include <vision_msgs/Detection3DArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <wsg_32_common/Move.h>
#include <std_srvs/Empty.h>
#include <grasp_dope/quintic_traj.h>
#include <tf/transform_listener.h>

#include <grasp_dope/goal_pose_plan_Action.h>
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

#include <geometric_shapes/shape_operations.h>
#include "slipping_control_common/Slipping_Control_Client.h"

bool success_planning_pp = false;
bool success;
double rate = 40; // Hz
vision_msgs::Detection3DArrayConstPtr box_size;
std::string object_name;
double mesh_scale;
std::string mesh_path;
std::vector<double> homing;

moveit::planning_interface::MoveGroupInterface::Plan planning_joint(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "end_effector_tool0");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_interface.setMaxVelocityScalingFactor(0.1); // 0.05
    move_group_interface.setPlanningTime(18);
    move_group_interface.setPlannerId("RRTstarkConfigDefault");

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_STREAM("Planning result: " << success);

    return my_plan;
}

auto planning_cartesian(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "end_effector_tool0");
    move_group_interface.setPlanningTime(2);
    // move_group_interface.setPoseReferenceFrame("end_effector_tool0");
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
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("/end_effector_tool0", "/tool0", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/end_effector_tool0", "/tool0",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    Eigen::Isometry3d T_TE; // omogeneous transfrom from tool0 to end-effector_tool0
    T_TE.translation().x() = transform.getOrigin().getX();
    T_TE.translation().y() = transform.getOrigin().getY();
    T_TE.translation().z() = transform.getOrigin().getZ();

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

auto attach_obj(std::string obj_id, std::string frame_id, moveit::planning_interface::MoveGroupInterface &move_group_interface, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, geometry_msgs::Pose pose_obj, float scale_obj)
{
    moveit_msgs::CollisionObject object_to_attach;

    scale_obj = scale_obj * mesh_scale;
    if (scale_obj > 0.80)
        scale_obj = 0.80;
    Eigen::Vector3d scale(scale_obj, scale_obj, scale_obj);

    object_to_attach.id = obj_id;

    // La posizione del box è definita rispetto alla terna world
    object_to_attach.header.frame_id = frame_id;

    shapes::Mesh *m = shapes::createMeshFromResource("file://" + mesh_path, scale);

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    object_to_attach.meshes.resize(1);
    object_to_attach.mesh_poses.resize(1);
    object_to_attach.meshes[0] = mesh;
    object_to_attach.mesh_poses[0] = pose_obj;

    object_to_attach.meshes.push_back(mesh);
    object_to_attach.mesh_poses.push_back(object_to_attach.mesh_poses[0]);

    object_to_attach.operation = object_to_attach.ADD;

    planning_scene_interface.applyCollisionObject(object_to_attach);
    return object_to_attach;
    // move_group_interface.attachObject(object_to_attach.id, "end_effector_tool0");
}

void execute_trajectory(const moveit_msgs::RobotTrajectory &my_plan, ros::NodeHandle &nh, bool scale)
{
    ros::Rate loop_rate(rate);
    ros::Publisher joint_cmd_pub = nh.advertise<sensor_msgs::JointState>("/motoman/joint_ll_control", 1);

    sensor_msgs::JointState joint_cmd;
    joint_cmd.position.resize(my_plan.joint_trajectory.points.at(0).positions.size());
    joint_cmd.velocity.resize(my_plan.joint_trajectory.points.at(0).positions.size());
    joint_cmd.name.resize(my_plan.joint_trajectory.points.at(0).positions.size());
    joint_cmd.header = my_plan.joint_trajectory.header;
    joint_cmd.name = my_plan.joint_trajectory.joint_names;

    // coeff_q1 coeff_q2 coeff_q3 ... coeff_q7
    //  a5
    //  a4
    //  ...
    //  a0
    Eigen::Matrix<double, 6, 7> coeff;

    auto num_points_traj = my_plan.joint_trajectory.points.size();
    double tf = 0;
    double t = 0;
    double t0 = 0;
    double scale_factor = 5.0; // 10

    ROS_INFO_STREAM("NUMERO DI PUNTI PIANIFICATI: " << num_points_traj);
    std::vector<sensor_msgs::JointState> joint_cmd_vect;
    for (int j = 0; j < num_points_traj - 1; j++)
    {
        tf = my_plan.joint_trajectory.points[j + 1].time_from_start.toSec() - my_plan.joint_trajectory.points[j].time_from_start.toSec();

        auto qi = my_plan.joint_trajectory.points[j].positions;
        auto qf = my_plan.joint_trajectory.points[j + 1].positions;

        auto qi_dot = my_plan.joint_trajectory.points[j].velocities;
        auto qf_dot = my_plan.joint_trajectory.points[j + 1].velocities;

        auto qi_dot_dot = my_plan.joint_trajectory.points[j].accelerations;
        auto qf_dot_dot = my_plan.joint_trajectory.points[j + 1].accelerations;

        if (scale)
        {
            tf = tf * scale_factor;
            for (int m = 0; m < qi.size(); m++)
            {
                qi_dot[m] = qi_dot[m] / scale_factor;
                qi_dot_dot[m] = qi_dot_dot[m] / pow(scale_factor, 2);

                qf_dot[m] = qf_dot[m] / scale_factor;
                qf_dot_dot[m] = qf_dot_dot[m] / pow(scale_factor, 2);
            }
        }

        update_coeff(coeff, tf, qi, qi_dot, qi_dot_dot, qf, qf_dot, qf_dot_dot, num_points_traj);

        t = 0;
        t0 = ros::Time::now().toSec();
        while (t <= tf)
        {
            t = ros::Time::now().toSec() - t0;
            for (int i = 0; i < qi.size(); i++)
            {
                joint_cmd.position.at(i) = quintic_q(t, coeff, i);
                joint_cmd.velocity.at(i) = 0.0; // quintic_qdot(t, coeff, i);
            }

            joint_cmd_pub.publish(joint_cmd);
            loop_rate.sleep();
        }
    }
}

bool executeCB(const grasp_dope::goal_pose_plan_GoalConstPtr &goal, actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> *as, ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state)
{
    /* Clear octomap to start planning */
    ros::service::waitForService("/clear_octomap");
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    if (!ros::service::call<std_srvs::Empty::Request, std_srvs::Empty::Response>("/clear_octomap", req, res))
    {
        ROS_INFO_STREAM("Error activating service clear_octomap...");
        return -1;
    }
    float scale_obj = goal->scale_obj;
    // create messages that are used to published feedback/result
    grasp_dope::goal_pose_plan_Feedback feedback;
    grasp_dope::goal_pose_plan_Result result;
    // bool success = true;

    geometry_msgs::Pose pre_grasp_pose;
    geometry_msgs::Pose grasp_pose;
    geometry_msgs::Pose post_grasp_pose;
    geometry_msgs::Pose pre_place_pose;
    geometry_msgs::Pose place_pose;
    geometry_msgs::Pose post_place_pose;

    moveit::core::RobotState start_state(*move_group_interface->getCurrentState());

    moveit::planning_interface::MoveGroupInterface::Plan plan_pre_grasp;
    moveit_msgs::RobotTrajectory plan_pick;
    moveit_msgs::RobotTrajectory plan_post_grasp;

    moveit::planning_interface::MoveGroupInterface::Plan plan_pre_place;
    moveit_msgs::RobotTrajectory plan_post_place;
    moveit_msgs::RobotTrajectory plan_place;

    moveit::planning_interface::MoveGroupInterface::Plan plan_homing;

    std::vector<geometry_msgs::Pose> pre_grasp_attemp_vector;
    geometry_msgs::Pose pre_grasp_attemp;
    std::vector<geometry_msgs::Pose> place_attempt;

    Eigen::Matrix3d rotation_start;
    Eigen::Matrix3d rotation_alpha;
    Eigen::Matrix3d rotation_theta;

    ros::NodeHandle temp;
    ros::Publisher pre_grasp_attempt_pub = temp.advertise<geometry_msgs::PoseStamped>("/attempt", 1);
    ros::Publisher pose_obj_pub = temp.advertise<geometry_msgs::PoseStamped>("/pose_obj_refined", 1);

    geometry_msgs::PoseStamped temp_pub;
    geometry_msgs::PoseStamped grasp_pose_stamped;

    std::vector<geometry_msgs::Pose> obj_pose_vector;
    geometry_msgs::Pose obj_pose_ = goal->goal_pose_pick.pose;

    double theta = 0.0;   // inclinazione rispetto all'oggetto
    double offset = 0.12; // offset pre-grasp

    if (object_name == "banana")
    {
        std::vector<double> cad_dimensions;
        cad_dimensions.resize(3);
        ros::param::get("/dope/dimensions/" + object_name, cad_dimensions);
        cad_dimensions.at(0) = cad_dimensions.at(0) * 0.01 * scale_obj;
        cad_dimensions.at(1) = cad_dimensions.at(1) * 0.01 * scale_obj;
        cad_dimensions.at(2) = cad_dimensions.at(2) * 0.01 * scale_obj;

        Eigen::Quaterniond orientation_OW(goal->goal_pose_pick.pose.orientation.w, goal->goal_pose_pick.pose.orientation.x, goal->goal_pose_pick.pose.orientation.y, goal->goal_pose_pick.pose.orientation.z);
        Eigen::Vector3d translation_OW(goal->goal_pose_pick.pose.position.x, goal->goal_pose_pick.pose.position.y, goal->goal_pose_pick.pose.position.z);
        Eigen::Matrix3d rotation_OW(orientation_OW);
        Eigen::Vector3d grasp_banana(0, -0.01, 0);
        Eigen::Vector3d tmp = rotation_OW * grasp_banana;
        // obj_pose_.position.x = obj_pose_.position.x + tmp(0);
        // obj_pose_.position.y = obj_pose_.position.y + tmp(1);
        // obj_pose_.position.z = obj_pose_.position.z + tmp(2);

        int inclination_attempt = 4;
        int points_attempt = 3;

        Eigen::Matrix3d rotation_GO;

        if (rotation_OW(2, 0) >= 0)
        {
            rotation_GO << 0, 0, -1,
                0, -1, 0,
                -1, 0, 0;
        }
        else
        {
            rotation_GO << 0, 0, 1,
                0, 1, 0,
                -1, 0, 0;
        }

        rotation_start = rotation_OW * rotation_GO;

        std::vector<double> attempt_position;
        float min_ = cad_dimensions.at(2) / 2 - cad_dimensions.at(2) / 4;
        float max_ = cad_dimensions.at(2) / 2 + cad_dimensions.at(2) / 4;

        for (int i = round(points_attempt / 2); i < points_attempt; i++)
        {
            attempt_position.push_back((min_ + i * (max_ - min_) / points_attempt) - cad_dimensions.at(2) / 2);
        }
        for (int i = 0; i <= round(points_attempt / 2); i++)
        {
            attempt_position.push_back((min_ + i * (max_ - min_) / points_attempt) - cad_dimensions.at(2) / 2);
        }

        for (int i = 0; i < attempt_position.size(); i++)
        {
            ROS_INFO_STREAM(attempt_position.at(i));
            Eigen::Vector3d grasp_banana(0, -0.01, attempt_position.at(i));
            tmp = rotation_OW * grasp_banana;
            obj_pose_.position.x = obj_pose_.position.x + tmp(0);
            obj_pose_.position.y = obj_pose_.position.y + tmp(1);
            obj_pose_.position.z = obj_pose_.position.z + tmp(2);
            obj_pose_.position.z = obj_pose_.position.z + 0.015;

            pre_grasp_attemp.position = obj_pose_.position;
            pre_grasp_attemp.position.z = pre_grasp_attemp.position.z + offset;

            for (int k = 1; k < inclination_attempt; k++)
            {
                theta = M_PI / 3 + (k * M_PI / (4 * inclination_attempt));
                if (rotation_OW(1, 2) < 0)
                {
                    theta = -theta;
                }

                /* Rotazione attorno all'asse y */
                rotation_theta << cos(theta), 0, sin(theta),
                    0, 1, 0,
                    -sin(theta), 0, cos(theta);

                Eigen::Quaterniond q_(rotation_start * rotation_theta);
                pre_grasp_attemp.orientation.w = q_.w();
                pre_grasp_attemp.orientation.x = q_.x();
                pre_grasp_attemp.orientation.y = q_.y();
                pre_grasp_attemp.orientation.z = q_.z();

                place_attempt.push_back(pre_grasp_attemp);

                pre_grasp_attemp_vector.push_back(pre_grasp_attemp);
                obj_pose_vector.push_back(obj_pose_);
            }
        }
    }

    else
    {
        double alpha = 0.0; // rotazione attorno all'oggetto

        rotation_start << 0, -1, 0,
            -1, 0, 0,
            0, 0, -1;
        int rotation_attempt = 6;
        int inclination_attempt = 4;
        for (int i = 2; i < inclination_attempt; i++)
        {
            theta = (i * M_PI / (3 * inclination_attempt));

            /* Rotazione attorno all'asse y */
            rotation_theta << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

            for (int k = 0; k < rotation_attempt; k++)
            {
                if (k < rotation_attempt / 2)
                    alpha = -(k * M_PI / rotation_attempt); //- M_PI_2;
                else
                    alpha = (k - floor(rotation_attempt / 2)) * M_PI / rotation_attempt + M_PI / rotation_attempt;

                pre_grasp_attemp.position.x = obj_pose_.position.x + offset * sin(theta) * sin(alpha);
                pre_grasp_attemp.position.y = obj_pose_.position.y + offset * sin(theta) * cos(alpha);
                pre_grasp_attemp.position.z = obj_pose_.position.z + offset * cos(theta);

                /* Matrice di rotazione attorno all'asse z */
                rotation_alpha << cos(alpha), -sin(alpha), 0,
                    sin(alpha), cos(alpha), 0,
                    0, 0, 1;

                Eigen::Quaterniond q_(rotation_start * rotation_alpha * rotation_theta);
                pre_grasp_attemp.orientation.w = q_.w();
                pre_grasp_attemp.orientation.x = q_.x();
                pre_grasp_attemp.orientation.y = q_.y();
                pre_grasp_attemp.orientation.z = q_.z();

                if (k == rotation_attempt / 2 - 1)
                {
                    for (int l = 0; l < rotation_attempt; l++)
                    {
                        place_attempt.push_back(pre_grasp_attemp);
                    }
                }

                pre_grasp_attemp_vector.push_back(pre_grasp_attemp);
                obj_pose_vector.push_back(obj_pose_);
            }
        }
    }

    int attempt = 0;
    std::vector<std::string> obj_id;
    obj_id.push_back(object_name);

    // Parameters for attach obj
    geometry_msgs::Pose attach_obj_pose;
    attach_obj_pose.position.x = goal->goal_pose_pick.pose.position.x;
    attach_obj_pose.position.y = goal->goal_pose_pick.pose.position.y;
    attach_obj_pose.position.z = goal->goal_pose_pick.pose.position.z;
    attach_obj_pose.orientation.w = goal->goal_pose_pick.pose.orientation.w;
    attach_obj_pose.orientation.x = goal->goal_pose_pick.pose.orientation.x;
    attach_obj_pose.orientation.y = goal->goal_pose_pick.pose.orientation.y;
    attach_obj_pose.orientation.z = goal->goal_pose_pick.pose.orientation.z;

    // auto object_ = attach_obj(object_name, "base_link", *move_group_interface, *planning_scene_interface, attach_obj_pose, scale_obj);

    // std::string topic_dope_pose = "/dope/pose_" + object_name;
    // auto dope_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(topic_dope_pose);

    // obj non ottimizato
    // attach_obj("dope_obj", dope_pose->header.frame_id, *move_group_interface, *planning_scene_interface, dope_pose->pose, 1);
    // move_group_interface->detachObject("dope_obj");

    while (!success_planning_pp && attempt < pre_grasp_attemp_vector.size())
    {
        ROS_INFO_STREAM(attempt + 1 << " ATTEMPT");

        temp_pub.pose = pre_grasp_attemp_vector.at(attempt);
        temp_pub.header.stamp = ros::Time::now();
        temp_pub.header.frame_id = "base_link";
        // std::cout << "Press Enter to publish frame";
        // std::cin.ignore();
        pre_grasp_attempt_pub.publish(temp_pub);

        pre_grasp_pose = pre_grasp_attemp_vector.at(attempt);

        grasp_pose = obj_pose_vector.at(attempt);
        grasp_pose.position.z = grasp_pose.position.z;
        grasp_pose_stamped.pose = grasp_pose;
        grasp_pose_stamped.header.frame_id = "base_link";
        grasp_pose_stamped.header.stamp = ros::Time::now();

        pose_obj_pub.publish(grasp_pose_stamped);
        grasp_pose.orientation = pre_grasp_attemp_vector.at(attempt).orientation;

        post_grasp_pose = pre_grasp_pose;

        place_pose.position = goal->goal_pose_place.pose.position;
        place_pose.orientation = place_attempt.at(attempt).orientation;

        pre_place_pose = place_pose;
        pre_place_pose.position.z = pre_place_pose.position.z + 0.10;

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
            // start_state.setFromIK(joint_model_group, get_tool_pose(pre_grasp_pose));
            start_state.setJointGroupPositions(joint_model_group, plan_pre_grasp.trajectory_.joint_trajectory.points.back().positions);
            move_group_interface->setStartState(start_state);

            plan_pick = planning_cartesian(get_tool_pose(grasp_pose), *move_group_interface);

            ROS_INFO_STREAM("Result planning grasp pose: " << success);
            if (success)
            {
                std::cout << "Press Enter to Continue";
                std::cin.ignore();
                // attach_obj(object_name, "base_link", *move_group_interface, *planning_scene_interface, attach_obj_pose, scale_obj);
                auto object_ = attach_obj(object_name, "base_link", *move_group_interface, *planning_scene_interface, attach_obj_pose, scale_obj);
                move_group_interface->attachObject(object_.id, "end_effector_tool0");
                /* Planning to post grasp pose */
                ROS_INFO_STREAM("--- Post grasp pose --- "
                                << "\n"
                                << post_grasp_pose);

                // start_state.setFromIK(joint_model_group, get_tool_pose(grasp_pose));
                start_state.setJointGroupPositions(joint_model_group, plan_pick.joint_trajectory.points.back().positions);
                move_group_interface->setStartState(start_state);

                plan_post_grasp = planning_cartesian(get_tool_pose(post_grasp_pose), *move_group_interface);
                ROS_INFO_STREAM("Result planning post grasp pose: " << success);

                if (success)
                {
                    std::cout << "Press Enter to Continue";
                    std::cin.ignore();
                    /* Planning to place pose */
                    ROS_INFO_STREAM("--- Pre-Place pose --- "
                                    << "\n"
                                    << pre_place_pose);

                    // start_state.setFromIK(joint_model_group, get_tool_pose(post_grasp_pose));
                    start_state.setJointGroupPositions(joint_model_group, plan_post_grasp.joint_trajectory.points.back().positions);
                    move_group_interface->setStartState(start_state);

                    plan_pre_place = planning_joint(pre_place_pose, *move_group_interface);
                    ROS_INFO_STREAM("Result planning place pose: " << success);
                    if (success)
                    {
                        std::cout << "Press ENTER to continue...";
                        std::cin.ignore();

                        ROS_INFO_STREAM("--- Place pose --- "
                                        << "\n"
                                        << place_pose);

                        // start_state.setFromIK(joint_model_group, get_tool_pose(post_grasp_pose));
                        start_state.setJointGroupPositions(joint_model_group, plan_pre_place.trajectory_.joint_trajectory.points.back().positions);
                        move_group_interface->setStartState(start_state);

                        plan_place = planning_cartesian(get_tool_pose(place_pose), *move_group_interface);

                        if (success)
                        {
                            std::cout << "Planning successfully completed. Press ENTER to continue...";
                            std::cin.ignore();

                            move_group_interface->detachObject(object_name);
                            success_planning_pp = true;
                            attempt = 0;
                        }
                        else
                        {
                            ROS_INFO_STREAM("Try with other pre-grasp pose...");
                            attempt = attempt + 1;
                            move_group_interface->detachObject(object_name);
                            planning_scene_interface->removeCollisionObjects(obj_id);
                        }
                    }
                    else
                    {
                        ROS_INFO_STREAM("Try with other pre-grasp pose...");
                        attempt = attempt + 1;
                        move_group_interface->detachObject(object_name);
                        planning_scene_interface->removeCollisionObjects(obj_id);
                    }
                }
                else
                {
                    ROS_INFO_STREAM("Try with other pre-grasp pose...");
                    attempt = attempt + 1;
                    move_group_interface->detachObject(object_name);
                    planning_scene_interface->removeCollisionObjects(obj_id);
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
            ROS_INFO_STREAM("Try with other pre-grasp pose...");
            attempt = attempt + 1;
        }
    }

    // ROS_INFO_STREAM("NUMERO DI PUNTI PIANIFICATI PRE GRASP: " << plan_pre_grasp.trajectory_.joint_trajectory.points.size());
    // std::cout << "ultimo pre grasp: " << std::endl;
    // for (auto element : plan_pre_grasp.trajectory_.joint_trajectory.points[plan_pre_grasp.trajectory_.joint_trajectory.points.size() - 1].positions)
    // {
    //     std::cout << element << std::endl;
    // }

    // ROS_INFO_STREAM("NUMERO DI PUNTI PIANIFICATI GRASP: " << plan_pick.joint_trajectory.points.size());
    // std::cout << "primo grasp: " << std::endl;
    // for (auto element : plan_pick.joint_trajectory.points[0].positions)
    // {
    //     std::cout << element << std::endl;
    // }
    // std::cout << "ultimo grasp: " << std::endl;
    // for (auto element : plan_pick.joint_trajectory.points[plan_pick.joint_trajectory.points.size() - 1].positions)
    // {
    //     std::cout << element << std::endl;
    // }

    // ROS_INFO_STREAM("NUMERO DI PUNTI PIANIFICATI POST GRASP: " << plan_post_grasp.joint_trajectory.points.size());
    // std::cout << "primo post grasp: " << std::endl;
    // for (auto element : plan_post_grasp.joint_trajectory.points[0].positions)
    // {
    //     std::cout << element << std::endl;
    // }
    // std::cout << "ultimo post grasp: " << std::endl;
    // for (auto element : plan_post_grasp.joint_trajectory.points[plan_post_grasp.joint_trajectory.points.size() - 1].positions)
    // {
    //     std::cout << element << std::endl;
    // }

    // ROS_INFO_STREAM("NUMERO DI PUNTI PIANIFICATI PLACE: " << plan_pre_place.trajectory_.joint_trajectory.points.size());
    // std::cout << "primo pre place: " << std::endl;
    // for (auto element : plan_pre_place.trajectory_.joint_trajectory.points[0].positions)
    // {
    //     std::cout << element << std::endl;
    // }
    // std::cout << "ultimo pre place: " << std::endl;
    // for (auto element : plan_pre_place.trajectory_.joint_trajectory.points.back().positions)
    // {
    //     std::cout << element << std::endl;
    // }

    // ROS_INFO_STREAM("NUMERO DI PUNTI PIANIFICATI PLACE: " << plan_place.joint_trajectory.points.size());
    // std::cout << "primo place: " << std::endl;
    // for (auto element : plan_place.joint_trajectory.points[0].positions)
    // {
    //     std::cout << element << std::endl;
    // }

    move_group_interface->detachObject(object_name);
    auto object_ = attach_obj(object_name, "base_link", *move_group_interface, *planning_scene_interface, attach_obj_pose, scale_obj);
    // planning_scene_interface->removeCollisionObjects(obj_id);
    result.success = success;
    as->setSucceeded(result);
    bool success_homing = false;
    Slipping_Control_Client slipping_control(*nh, true, true);
    double grasp_force = 1.0; //[N]
    slipping_control.home();

    if (success_planning_pp)
    {
        std::cout << "Press Enter to start executing";
        std::cin.ignore();
        ROS_INFO_STREAM("Executing trajectory pre_grasp...");
        execute_trajectory(plan_pre_grasp.trajectory_, *nh, false);

        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("Executing trajectory pick...");
        execute_trajectory(plan_pick, *nh, true);
        std::cout << "Press Enter to Continue with grasp";
        std::cin.ignore();

        slipping_control.grasp(grasp_force);
        slipping_control.slipping_avoidance();
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        // attach_obj(object_name, "base_link", *move_group_interface, *planning_scene_interface, attach_obj_pose, scale_obj);
        move_group_interface->attachObject(object_.id, "end_effector_tool0");

        ROS_INFO_STREAM("Executing trajectory post grasp...");
        execute_trajectory(plan_post_grasp, *nh, true);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("Executing trajectory pre place...");
        execute_trajectory(plan_pre_place.trajectory_, *nh, false);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("Executing trajectory place...");
        execute_trajectory(plan_place, *nh, true);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("--- OPENING GRIPPER ---");
        slipping_control.home();

        move_group_interface->detachObject(object_name);

        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        move_group_interface->setStartStateToCurrentState();
        post_place_pose = place_pose;
        post_place_pose.position.z = post_place_pose.position.z + 0.11;
        plan_post_place = planning_cartesian(get_tool_pose(post_place_pose), *move_group_interface);
        if (success)
        {
            std::cout << "Press Enter to Continue";
            std::cin.ignore();
            execute_trajectory(plan_post_place, *nh, true);
            std::cout << "Press Enter to Continue";
            std::cin.ignore();
            move_group_interface->setStartStateToCurrentState();
            move_group_interface->setJointValueTarget(homing);
            move_group_interface->setMaxVelocityScalingFactor(0.05);
            move_group_interface->setMaxAccelerationScalingFactor(0.05);

            success_homing = (move_group_interface->plan(plan_homing) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success_homing)
            {
                std::cout << "Press Enter to return Home";
                std::cin.ignore();
                execute_trajectory(plan_homing.trajectory_, *nh, false);
            }
        }
        else
        {
            ROS_INFO_STREAM("Error planning");
        }
    }

    planning_scene_interface->removeCollisionObjects(obj_id);
    success_planning_pp = false;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_server");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::param::get("/dope/object_of_interest", object_name);
    const std::string mesh_scale_string = "/dope/mesh_scales/" + object_name;
    const std::string mesh_path_string = "/dope/meshes/" + object_name;
    ros::param::get(mesh_path_string, mesh_path);
    ros::param::get(mesh_scale_string, mesh_scale);

    /* Parametri per la costruzione della scena */
    static const std::string PLANNING_GROUP = "yaskawa_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    auto start_joints_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/motoman/joint_states");
    // auto start_joints_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    std::vector<double> joints_values;

    for (auto element : start_joints_values->position)
    {
        joints_values.push_back(element);
        ROS_INFO_STREAM(element);
    }
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    // std::vector<double> joints_values_tmp;
    // for (int i = 0; i < 7; i++)
    // {
    //     joints_values_tmp.push_back(joints_values[i]);
    // }
    // kinematic_state->setJointGroupPositions(joint_model_group, joints_values_tmp);

    kinematic_state->setJointGroupPositions(joint_model_group, joints_values);
    homing = move_group_interface.getCurrentJointValues();

    for (auto element : move_group_interface.getCurrentJointValues())
    {
        ROS_INFO_STREAM(element);
    }

    ros::Duration timeout_box_size(1);
    /* Subscriber to read size of detected obj */
    box_size = ros::topic::waitForMessage<vision_msgs::Detection3DArray>("/dope/detected_objects");

    /* Creazione del ros action */
    actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> as(nh, "planning_action", boost::bind(&executeCB, _1, &as, &nh, &move_group_interface, &planning_scene_interface, joint_model_group, kinematic_state), false); // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    as.start();
    ROS_INFO_STREAM("Waiting for action client..");

    ros::waitForShutdown();

    return 0;
}