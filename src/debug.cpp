bool executeCB(const grasp_dope::goal_pose_plan_GoalConstPtr &goal, actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> *as, ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state)
{
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state = move_group_interface->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[1] = joint_group_positions[1] + 0.17; // -1/6 turn in radians
    move_group_interface->setJointValueTarget(joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group_interface->setMaxVelocityScalingFactor(0.01);
    move_group_interface->setMaxAccelerationScalingFactor(0.01);

    success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        

        std::cout << "Press Enter to start executing";
        std::cin.ignore();

        sensor_msgs::JointState joint_cmd;
        joint_cmd.position.resize(my_plan.trajectory_.joint_trajectory.points.at(0).positions.size());
        joint_cmd.velocity.resize(my_plan.trajectory_.joint_trajectory.points.at(0).positions.size());
        joint_cmd.name.resize(my_plan.trajectory_.joint_trajectory.points.at(0).positions.size());
        joint_cmd.header = my_plan.trajectory_.joint_trajectory.header;
        joint_cmd.name = my_plan.trajectory_.joint_trajectory.joint_names;

        auto num_points_traj = my_plan.trajectory_.joint_trajectory.points.size();

        for (int j = 0; j < num_points_traj - 1; j++)
        {
            auto qi = my_plan.trajectory_.joint_trajectory.points.at(j).positions;
            auto qf = my_plan.trajectory_.joint_trajectory.points.at(j + 1).positions;

            auto qi_dot = my_plan.trajectory_.joint_trajectory.points.at(j).velocities;
            auto qf_dot = my_plan.trajectory_.joint_trajectory.points.at(j + 1).velocities;

            for (int k = 0; k <= rate; k++)
            {
                double tau = k / rate;
                for (int i = 0; i < qi.size(); i++)
                {
                    joint_cmd.position.at(i) = quintic_q(tau, qi[i], qf[i]);
                    ROS_INFO_STREAM("terzo for before qdot");
                    joint_cmd.velocity.at(i) = quintic_qdot(tau, qi_dot[i], qf_dot[i]);
                }

                joint_cmd_pub.publish(joint_cmd);
                loop_rate.sleep();
            }
        }

        // move_group_interface->execute(plan_pre_grasp);

        std::cout << "Press Enter to Continue";
        std::cin.ignore();
    }
    return success;
}