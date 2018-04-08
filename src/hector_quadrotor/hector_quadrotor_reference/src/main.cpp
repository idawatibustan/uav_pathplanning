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
    std::string obsAdr, refAdr;
    n.getParam("/refPub/obsAdr", obsAdr="noFile");
    n.getParam("/refPub/refAdr", refAdr="noFile");
    std::cout<<"Read files from: "<<std::endl;
    std::cout<<obsAdr<<std::endl;
    std::cout<<refAdr<<std::endl;
    ros::Publisher init_pos_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",5);
    ros::Publisher ref_pos_pub = n.advertise<geometry_msgs::PoseStamped>("command/pose",1);
    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("shutdown", eReq, eRes);
    ros::Duration(3).sleep(); 
    ros::Rate loop_rate(20);

    /* Read obstacle position txt file*/
    std::ifstream obsFile;
    float obsout[2];
    obsFile.open(obsAdr.c_str());
    for (int i=0; i<3;i++)
    {
        if(obsFile >> obsout[0] >> obsout[1])
	{
	    gazebo_msgs::ModelState msgMS;
	    std::string str = "unit_cylinder_";	
	    str.append(static_cast<std::ostringstream*>( &(std::ostringstream() << i+1) )->str());
            msgMS.model_name = str;
            msgMS.pose.position.x = obsout[0];
            msgMS.pose.position.y = obsout[1];
            msgMS.pose.position.z = 0.5;
            init_pos_pub.publish(msgMS);
	}
    }
    /* Read the reference txt file*/
    std::ifstream refFile;
    refFile.open(refAdr.c_str());

    float refout[11];


    int count = 0;
    while (ros::ok())
    {
        ++count;
	
        if (refFile >> refout[0]>> refout[1]>> refout[2]>> refout[3]>> refout[4]>> refout[5]
                >> refout[6]>> refout[7]>> refout[8]>> refout[9]>> refout[10])
        {	
	    if(count == 1)
            {
		//Set the model position on the ground
                gazebo_msgs::ModelState msgMS;	
                msgMS.model_name = "quadrotor";
                msgMS.pose.position.x = refout[0];
                msgMS.pose.position.y = refout[1];
                msgMS.pose.position.z = 0;
                init_pos_pub.publish(msgMS);
		ros::Duration(1).sleep();

		//Set the position reference on the ground
		geometry_msgs::PoseStamped msgGe;
                msgGe.pose.position.x = refout[0];
                msgGe.pose.position.y = refout[1];
                msgGe.pose.position.z = 0;
		ref_pos_pub.publish(msgGe);
		ros::Duration(1).sleep();
		ros::service::call("engage", eReq, eRes);
	    }
	    else
	    {
		geometry_msgs::PoseStamped msgGe;
                msgGe.pose.position.x = refout[0];
                msgGe.pose.position.y = refout[1];
                msgGe.pose.position.z = refout[2];
		ref_pos_pub.publish(msgGe);
	    }
        }
        ros::spinOnce();

        loop_rate.sleep();

    }


    return 0;
}
