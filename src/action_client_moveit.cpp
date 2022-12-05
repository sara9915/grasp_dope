#include "ros/ros.h"
#include <std_srvs/Trigger.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_action");
    ros::NodeHandle nh;

    /* Invocazione del servizio per la costruzione della scena */
    ros::service::waitForService("/build_scene");
    std_srvs::Trigger::Request req;
    req = {};
    std_srvs::Trigger::Response res;
    if (!ros::service::call<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("/build_scene", req, res))
    {
        ROS_INFO_STREAM("Error Creating scene...");
        return -1;
    }

    //ros::spin();
    return 0;
}