#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf2_static_gripper");

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // WGS-32_Body_2 to Base_Jaw_4
    geometry_msgs::TransformStamped static_transformStamped1;
    static_transformStamped1.header.stamp = ros::Time::now();
    static_transformStamped1.header.frame_id = "WSG-32_Body_2";
    static_transformStamped1.child_frame_id = "Base_Jaw_4";
    static_transformStamped1.transform.translation.x = -0.045628;
    static_transformStamped1.transform.translation.y = -0.160;
    static_transformStamped1.transform.translation.z = -0.026;
    static_transformStamped1.transform.rotation.x = 0;
    static_transformStamped1.transform.rotation.y = 0;
    static_transformStamped1.transform.rotation.z = 0;
    static_transformStamped1.transform.rotation.w = 1;
    static_broadcaster.sendTransform(static_transformStamped1);

    // WGS-32_Body_2 to Base_Jaw_3
    geometry_msgs::TransformStamped static_transformStamped2;
    static_transformStamped2.header.stamp = ros::Time::now();
    static_transformStamped2.header.frame_id = "WSG-32_Body_2";
    static_transformStamped2.child_frame_id = "Base_Jaw_3";
    static_transformStamped2.transform.translation.x = -0.050628;
    static_transformStamped2.transform.translation.y = -0.160;
    static_transformStamped2.transform.translation.z = -0.049;
    static_transformStamped2.transform.rotation.x = 0;
    static_transformStamped2.transform.rotation.y = 0;
    static_transformStamped2.transform.rotation.z = 0;
    static_transformStamped2.transform.rotation.w = 1;
    static_broadcaster.sendTransform(static_transformStamped2);

    ros::spin();
    return 0;
};