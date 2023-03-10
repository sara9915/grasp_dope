#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#define ORIGIN_ 0.068
double position = 0.0;
geometry_msgs::TransformStamped wsg_jaw3;
geometry_msgs::TransformStamped wsg_jaw4;

void executeCB_gripper(const sensor_msgs::JointState::ConstPtr &msg)
{
    ROS_INFO_STREAM(msg->position.at(0));
    position = msg->position.at(0);

    double base_jaw_4_x = ORIGIN_ / 2 - position / 2;
    double base_jaw_3_x = -ORIGIN_ / 2 + position / 2;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // WGS-32_Body_2 to Base_Jaw_4

    wsg_jaw4.header.stamp = ros::Time::now();
    wsg_jaw4.header.frame_id = "WSG-32_Body_2";
    wsg_jaw4.child_frame_id = "Base_Jaw_4";
    wsg_jaw4.transform.translation.x = base_jaw_4_x; // 0.002499;
    wsg_jaw4.transform.translation.y = -0.031;
    wsg_jaw4.transform.translation.z = -0.024;
    wsg_jaw4.transform.rotation.x = 0;
    wsg_jaw4.transform.rotation.y = 0;
    wsg_jaw4.transform.rotation.z = 0;
    wsg_jaw4.transform.rotation.w = 1;
    // static_broadcaster.sendTransform(wsg_jaw4);

    // WGS-32_Body_2 to Base_Jaw_3

    wsg_jaw3.header.stamp = ros::Time::now();
    wsg_jaw3.header.frame_id = "WSG-32_Body_2";
    wsg_jaw3.child_frame_id = "Base_Jaw_3";
    wsg_jaw3.transform.translation.x = base_jaw_3_x; // 0.059;
    wsg_jaw3.transform.translation.y = -0.031;
    wsg_jaw3.transform.translation.z = -0.048;
    wsg_jaw3.transform.rotation.x = 0;
    wsg_jaw3.transform.rotation.y = 0;
    wsg_jaw3.transform.rotation.z = 0;
    wsg_jaw3.transform.rotation.w = 1;
    // static_broadcaster.sendTransform(wsg_jaw3);
}

void executeCB_full(const sensor_msgs::JointState::ConstPtr &msg, ros::Publisher *full_joints_pub)
{
    sensor_msgs::JointState full_joints;
    for(int i = 0 ; i < msg->position.size(); i++)
    {
        full_joints.name.push_back(msg->name.at(i));
        full_joints.position.push_back(msg->position.at(i));
    }
    
    full_joints.name.push_back("right_joint");
    full_joints.name.push_back("left_joint");
    full_joints.position.push_back(wsg_jaw3.transform.translation.x);
    full_joints.position.push_back(wsg_jaw4.transform.translation.x);
    full_joints.header.stamp = ros::Time::now();
    full_joints_pub->publish(full_joints);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf2_static_gripper");
    ros::NodeHandle nh;

    ros::Publisher full_joints_pub = nh.advertise<sensor_msgs::JointState>("/full_joint_states", 1);
    ros::Subscriber sub_gripper = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, boost::bind(&executeCB_gripper, _1));
    ros::Subscriber sub_full = nh.subscribe<sensor_msgs::JointState>("/motoman/joint_states", 1, boost::bind(&executeCB_full, _1, &full_joints_pub));

    ros::spin();
    return 0;
};