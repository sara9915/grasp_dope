#include <math.h> /* pow */
#include "ros/ros.h"
#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_interp");
    ros::NodeHandle nh;

    {
        double qi = 0;
        double qf = 0.5;

        double qi_dot = 0.0;
        double qf_dot = 1.0;

        double qi_dot_dot = 2.0;
        double qf_dot_dot = 0.0;

        double q;
        double q_dot;
        double q_dot_dot;

        std_msgs::Float64 q_msg;
        std_msgs::Float64 qdot_msg;

        double t = 0;
        double t0 = ros::Time::now().toSec();
        double tf = 0;

        if (qi_dot_dot != 0 || qf_dot_dot != 0)
        {
            tf = qf_dot - qi_dot / (qf_dot_dot - qi_dot_dot);
        }
        else // (qi_dot == qf_dot)
        {
            tf = (qf - qi) / (qi_dot);
        }
        // else
        // {
        //     tf = (qf - qi) / (qf_dot - qi_dot);
        // }

        double rate = 50;
        ros::Rate loop_rate(rate);

        ros::Publisher q_pub = nh.advertise<std_msgs::Float64>("/traj_interp_q", 1);
        ros::Publisher qdot_pub = nh.advertise<std_msgs::Float64>("/traj_interp_qdot", 1);

        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        t0 = ros::Time::now().toSec();
        sun::Quintic_Poly_Traj quintic_traj(tf, qi, qf, t0, qi_dot, qf_dot, qi_dot_dot, qf_dot_dot);
        while (quintic_traj.getTimeLeft(ros::Time::now().toSec()) > 0)
        {
            t = ros::Time::now().toSec();
            q = quintic_traj.getPosition(t);
            q_dot = quintic_traj.getVelocity(t);

            q_msg.data = q;
            qdot_msg.data = q_dot;
            q_pub.publish(q_msg);
            qdot_pub.publish(qdot_msg);

            loop_rate.sleep();
        }
    }

    {
        ROS_INFO_STREAM("2");
        double qi = 0.5;
        double qf = 1;

        double qi_dot = 1;
        double qf_dot = 1;

        double qi_dot_dot = 0.0;
        double qf_dot_dot = 0.0;

        double q;
        double q_dot;
        double q_dot_dot;

        std_msgs::Float64 q_msg;
        std_msgs::Float64 qdot_msg;

        double t = 0;
        double t0 = ros::Time::now().toSec();
        double tf = 0;

        if (qi_dot_dot != 0 || qf_dot_dot != 0)
        {
            tf = qf_dot - qi_dot / (qf_dot_dot - qi_dot_dot);
        }
        else// (qi_dot == qf_dot)
        {
            tf = (qf - qi) / (qi_dot);
        }
        // else
        // {
        //     tf = (qf - qi) / (qf_dot - qi_dot);
        // }

        double rate = 50;
        ros::Rate loop_rate(rate);

        ros::Publisher q_pub = nh.advertise<std_msgs::Float64>("/traj_interp_q", 1);
        ros::Publisher qdot_pub = nh.advertise<std_msgs::Float64>("/traj_interp_qdot", 1);

        t0 = ros::Time::now().toSec();
        sun::Quintic_Poly_Traj quintic_traj(tf, qi, qf, t0, qi_dot, qf_dot, qi_dot_dot, qf_dot_dot);
        while (quintic_traj.getTimeLeft(ros::Time::now().toSec()) > 0)
        {
            t = ros::Time::now().toSec();
            q = quintic_traj.getPosition(t);
            q_dot = quintic_traj.getVelocity(t);

            q_msg.data = q;
            qdot_msg.data = q_dot;
            q_pub.publish(q_msg);
            qdot_pub.publish(qdot_msg);

            loop_rate.sleep();
        }
    }

    {
        ROS_INFO_STREAM("3");
        double qi = 1;
        double qf = 1.5;

        double qi_dot = 1.0;
        double qf_dot = 0.0;

        double qi_dot_dot = 0.0;
        double qf_dot_dot = -2.0;

        double q;
        double q_dot;
        double q_dot_dot;

        std_msgs::Float64 q_msg;
        std_msgs::Float64 qdot_msg;

        double t = 0;
        double t0 = ros::Time::now().toSec();
        double tf = 0;

        if (qi_dot_dot != 0 || qf_dot_dot != 0)
        {
            tf = qf_dot - qi_dot / (qf_dot_dot - qi_dot_dot);
        }
        else // (qi_dot == qf_dot)
        {
            tf = (qf - qi) / (qi_dot);
        }
        // else
        // {
        //     tf = (qf - qi) / (qf_dot - qi_dot);
        // }

        double rate = 50;
        ros::Rate loop_rate(rate);

        ros::Publisher q_pub = nh.advertise<std_msgs::Float64>("/traj_interp_q", 1);
        ros::Publisher qdot_pub = nh.advertise<std_msgs::Float64>("/traj_interp_qdot", 1);

        t0 = ros::Time::now().toSec();
        sun::Quintic_Poly_Traj quintic_traj(tf, qi, qf, t0, qi_dot, qf_dot, qi_dot_dot, qf_dot_dot);
        while (quintic_traj.getTimeLeft(ros::Time::now().toSec()) > 0)
        {
            t = ros::Time::now().toSec();
            q = quintic_traj.getPosition(t);
            q_dot = quintic_traj.getVelocity(t);

            q_msg.data = q;
            qdot_msg.data = q_dot;
            q_pub.publish(q_msg);
            qdot_pub.publish(qdot_msg);

            loop_rate.sleep();
        }
    }

    return 0;
}