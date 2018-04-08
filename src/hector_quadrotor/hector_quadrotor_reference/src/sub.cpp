#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <std_srvs/Empty.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "refPub");
    ros::NodeHandle n;
    std::string refAdr;
    n.getParam("/refPub/refAdr", refAdr="noFile");
    ros::Publisher ref_pos_pub = n.advertise<geometry_msgs::PoseStamped>("command/pose",1);

    ros::Duration(3).sleep(); 
    ros::Rate loop_rate(20);

    /* Read the reference txt file*/
    std::ifstream refFile;
    refFile.open(refAdr.c_str());

    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("engage", eReq, eRes);

    float refout[11];
    int count = 0;
    while (ros::ok())
    {
        ++count;
	
        if (refFile >> refout[0]>> refout[1]>> refout[2]>> refout[3]>> refout[4]>> refout[5]
                >> refout[6]>> refout[7]>> refout[8]>> refout[9]>> refout[10])
        {	
	    geometry_msgs::PoseStamped msgGe;
            msgGe.pose.position.x = refout[0];
            msgGe.pose.position.y = -refout[1];
            msgGe.pose.position.z = -refout[2];
	    ref_pos_pub.publish(msgGe);
        }
        ros::spinOnce();

        loop_rate.sleep();

    }


    return 0;
}
