#include <math.h> /* pow */
#include "ros/ros.h"
#include <vision_msgs/Detection3DArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <wsg_32_common/Move.h>
#include <std_srvs/Empty.h>

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
double rate = 50; // Hz
vision_msgs::Detection3DArrayConstPtr box_size;
std::vector<double> homing;

moveit::planning_interface::MoveGroupInterface::Plan planning_joint(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "end_effector_tool0");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setPlannerId("RRTstarkConfigDefault");

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_STREAM("Planning result: " << success);

    return my_plan;
}

auto planning_cartesian(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "end_effector_tool0");
    move_group_interface.setMaxVelocityScalingFactor(0.005);
    move_group_interface.setMaxAccelerationScalingFactor(0.005);
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
    box_primitive.dimensions[primitive.BOX_X] = box_size->detections[0].bbox.size.x;
    box_primitive.dimensions[primitive.BOX_Y] = box_size->detections[0].bbox.size.y;
    box_primitive.dimensions[primitive.BOX_Z] = box_size->detections[0].bbox.size.z;

    // La posizione del box ?? definita rispetto alla terna world
    object_to_attach.header.frame_id = "base_link";

    // First, we add the object to the world (without using a vector)
    object_to_attach.primitives.push_back(box_primitive);
    Eigen::Matrix3d rotation;
    rotation << 1,0,0,
                0,1,0,
                0,0,0;
    Eigen::Quaterniond quat(rotation);
    pose_obj.orientation.w = quat.w();
    pose_obj.orientation.x = quat.x();
    pose_obj.orientation.y = quat.y();
    pose_obj.orientation.z = quat.z();

    object_to_attach.primitive_poses.push_back(pose_obj);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    move_group_interface.attachObject(object_to_attach.id, "end_effector_tool0");
}

void update_coeff(Eigen::Matrix<double, 6, 7> &coeff, const double &tf, const std::vector<double> &qi, const std::vector<double> &qi_dot, const std::vector<double> &qi_dot_dot, const std::vector<double> &qf, const std::vector<double> &qf_dot, const std::vector<double> &qf_dot_dot, const double &num_points_traj)
{
    double tf_2 = pow(tf, 2);
    double tf_3 = pow(tf, 3);
    double tf_4 = pow(tf, 4);
    double tf_5 = pow(tf, 5);

    Eigen::Matrix3d A;
    A << 20.0 * tf_3, 12.0 * tf_2, 6.0 * tf,
        5.0 * tf_4, 4.0 * tf_3, 3.0 * tf_2,
        tf_5, tf_4, tf_3;

    Eigen::Vector3d b;
    Eigen::Vector3d coeff_calc;

    for (int j = 0; j < coeff.cols(); j++)
    {
        coeff(5, j) = qi[j];               // a0
        coeff(4, j) = qi_dot[j];           // a1
        coeff(3, j) = qi_dot_dot[j] / 2.0; // a2

        b(0) = qf_dot_dot[j] - qi_dot_dot[j];
        b(1) = qf_dot[j] - qi_dot[j] - qi_dot_dot[j] * tf;
        b(2) = qf[j] - qi[j] - qi_dot[j] * tf - qi_dot_dot[j] * tf_2 / 2.0;

        coeff_calc = A.inverse() * b;

        coeff(0, j) = coeff_calc[0]; // a5
        coeff(1, j) = coeff_calc[1]; // a4
        coeff(2, j) = coeff_calc[2]; // a3
    }
}

double quintic_q(const double &t, const Eigen::Matrix<double, 6, 7> &coeff, const int &i)
{
    double t_2 = pow(t, 2);
    double t_3 = pow(t, 3);
    double t_4 = pow(t, 4);
    double t_5 = pow(t, 5);

    return (coeff(0, i) * t_5 + coeff(1, i) * t_4 + coeff(2, i) * t_3 + coeff(3, i) * t_2 + coeff(4, i) * t + coeff(5, i));
}

double quintic_qdot(const double &t, const Eigen::Matrix<double, 6, 7> &coeff, const int &i)
{
    double t_2 = pow(t, 2);
    double t_3 = pow(t, 3);
    double t_4 = pow(t, 4);
    double t_5 = pow(t, 5);

    return (coeff(0, i) * 5 * t_4 + coeff(1, i) * 4 * t_3 + coeff(2, i) * 3 * t_2 + coeff(3, i) * 2 * t + coeff(4, i));
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
    double scale_factor = 10.0;

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
                joint_cmd.velocity.at(i) = quintic_qdot(t, coeff, i);
            }

            joint_cmd_pub.publish(joint_cmd);
            loop_rate.sleep();
        }
    }
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
    geometry_msgs::Pose post_place_pose;

    moveit::core::RobotState start_state(*move_group_interface->getCurrentState());

    moveit::planning_interface::MoveGroupInterface::Plan plan_pre_grasp;
    moveit_msgs::RobotTrajectory plan_pick;
    moveit_msgs::RobotTrajectory plan_post_grasp;

    moveit::planning_interface::MoveGroupInterface::Plan plan_place;

    moveit_msgs::RobotTrajectory plan_post_place;
    moveit::planning_interface::MoveGroupInterface::Plan plan_homing;

    std::vector<geometry_msgs::Pose> pre_grasp_attemp_vector;
    geometry_msgs::Pose pre_grasp_attemp;

    Eigen::Matrix3d rotation_start;
    Eigen::Matrix3d rotation_alpha;
    Eigen::Matrix3d rotation_theta;

    double alpha = 0.0; // rotazione attorno all'oggetto
    double theta = 0.0; // inclinazione rispetto all'oggetto
    double offset = 0.08; //offset pre-grasp

    rotation_start << 0, -1, 0,
        -1, 0, 0,
        0, 0, -1;
    int rotation_attempt = 4;
    int inclination_attempt = 4;
    ros::NodeHandle temp;
    ros::Publisher pre_grasp_attempt_pub = temp.advertise<geometry_msgs::PoseStamped>("/attempt", 1);
    geometry_msgs::PoseStamped temp_pub;

    for (int i = 1; i < inclination_attempt; i++)
    {
        theta = (i * M_PI / (3 * inclination_attempt));
        // ROS_INFO_STREAM("theta: " << theta);

        /* Rotazione attorno all'asse y */
        rotation_theta << cos(theta), 0, sin(theta),
            0, 1, 0,
            -sin(theta), 0, cos(theta);

        for (int k = 0; k < rotation_attempt; k++)
        {
            alpha = (k * 2 * M_PI / rotation_attempt);
            // ROS_INFO_STREAM("alpha: " << alpha);
            pre_grasp_attemp.position.x = goal->goal_pose_pick.pose.position.x + offset * sin(theta) * sin(alpha);
            pre_grasp_attemp.position.y = goal->goal_pose_pick.pose.position.y + offset * sin(theta) * cos(alpha);
            pre_grasp_attemp.position.z = goal->goal_pose_pick.pose.position.z + offset * cos(theta);

            /* Matrice di rotazione attorno all'asse z */
            rotation_alpha << cos(alpha), -sin(alpha), 0,
                sin(alpha), cos(alpha), 0,
                0, 0, 1;

            Eigen::Quaterniond q_(rotation_start * rotation_alpha * rotation_theta);
            pre_grasp_attemp.orientation.w = q_.w();
            pre_grasp_attemp.orientation.x = q_.x();
            pre_grasp_attemp.orientation.y = q_.y();
            pre_grasp_attemp.orientation.z = q_.z();

            pre_grasp_attemp_vector.push_back(pre_grasp_attemp);
        }
    }

    int attempt = 0;
    std::vector<std::string> obj_id;
    obj_id.push_back("obj");

    while (!success_planning_pp && attempt < pre_grasp_attemp_vector.size())
    {
        ROS_INFO_STREAM(attempt + 1 << " ATTEMPT");

        temp_pub.pose = pre_grasp_attemp_vector.at(attempt);
        temp_pub.header.stamp = ros::Time::now();
        temp_pub.header.frame_id = "base_link";
        pre_grasp_attempt_pub.publish(temp_pub);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        pre_grasp_pose = pre_grasp_attemp_vector.at(attempt);

        grasp_pose = goal->goal_pose_pick.pose;
        grasp_pose.orientation = pre_grasp_attemp_vector.at(attempt).orientation;

        post_grasp_pose = pre_grasp_pose;

        place_pose = goal->goal_pose_place.pose;
        place_pose.orientation = pre_grasp_attemp_vector.at(attempt).orientation;

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
                        planning_scene_interface->removeCollisionObjects(obj_id);
                        success_planning_pp = true;
                        attempt = 0;
                    }
                    else
                    {
                        ROS_INFO_STREAM("Try with other pre-grasp pose...");
                        attempt = attempt + 1;
                        move_group_interface->detachObject("obj");
                        planning_scene_interface->removeCollisionObjects(obj_id);
                    }
                }
                else
                {
                    ROS_INFO_STREAM("Try with other pre-grasp pose...");
                    attempt = attempt + 1;
                    move_group_interface->detachObject("obj");
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

    result.success = success;
    as->setSucceeded(result);
    bool success_homing = false;

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
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("--- CLOSING GRIPPER ---");
        ros::service::waitForService("/wsg_32_driver/grasp");
        wsg_32_common::Move::Request req;
        wsg_32_common::Move::Response res;

        req.width = 38.0;
        req.speed = 10.0;

        if (!ros::service::call<wsg_32_common::Move::Request, wsg_32_common::Move::Response>("/wsg_32_driver/grasp", req, res))
        {
            ROS_INFO_STREAM("Error activating service move gripper...");
            return -1;
        }

        attach_obj(*move_group_interface, *planning_scene_interface, goal->goal_pose_pick.pose);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("Executing trajectory post grasp...");
        execute_trajectory(plan_post_grasp, *nh, true);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("Executing trajectory place...");
        execute_trajectory(plan_place.trajectory_, *nh, false);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("--- OPENING GRIPPER ---");
        ros::service::waitForService("/wsg_32_driver/homing");
        std_srvs::Empty::Request req_home;
        std_srvs::Empty::Response res_home;
        req_home = {};

        if (!ros::service::call<std_srvs::Empty::Request, std_srvs::Empty::Response>("/wsg_32_driver/homing", req_home, res_home))
        {
            ROS_INFO_STREAM("Error activating service move gripper...");
            return -1;
        }
        move_group_interface->detachObject("obj");

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

    auto start_joints_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/motoman/joint_states");
    std::vector<double> joints_values;
    for (auto element : start_joints_values->position)
    {
        joints_values.push_back(element);
        ROS_INFO_STREAM(element);
    }
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

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

    ros::waitForShutdown();

    return 0;
}