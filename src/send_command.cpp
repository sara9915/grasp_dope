#include "ros/ros.h"

#include <memory>
#include <std_srvs/Trigger.h>

#include <ros/callback_queue.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "grasp_dope/start_reading.h"
#include <sensor_msgs/JointState.h>

bool start = false;

sensor_msgs::JointState jointCommand;

bool start_reading_cmd(grasp_dope::start_reading::Request  &req,
         grasp_dope::start_reading::Response &res)
{
  res.state = req.activate;
  start = res.state;
  if(res.state == true) ROS_INFO_STREAM("Service start_receive_cmd: ACTIVATED");
  else ROS_INFO_STREAM("Service start_receive_cmd: DEACTIVATED");
  return true;
}

void readCommand(const sensor_msgs::JointStateConstPtr& jointCommandMsg)
{
	jointCommand = *jointCommandMsg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "receive_cmd");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("receive_cmd", start_reading_cmd);
    
    /* Publisher.*/
    ros::Publisher joint_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/motoman/joint_ll_control", 1);

    /* Subscriber.*/
    ros::Subscriber ll_control_sub = nh.subscribe("/joint_states", 1, readCommand);
    
    while(ros::ok())
    {
        if(start == true) joint_cmd_pub.publish(jointCommand);
        ros::spinOnce();
    }

    return 0;
}
