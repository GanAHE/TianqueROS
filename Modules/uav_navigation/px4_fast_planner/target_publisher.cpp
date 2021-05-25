//Publish the topic of '/person_info', we will define the type: 'myCodeControlTrutlesim::Person'

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline/non_uniform_bspline.h>
#include <iostream>
#include <path_searching/topo_prm.h>
#include <plan_env/obj_predictor.h>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"person_publisher");

    ros::NodeHandle n;

    ros::Publisher person_info_pub = n.advertise<myCodeControlTurtlesim::Person>("/person_info",10);

    ros::Rate loop_rate(1);
    int cout = 0;

      visualization_msgs::Marker mk;
	mk.header.frame_id = "world";
	mk.header.stamp    = ros::Time::now();
	mk.type            = visualization_msgs::Marker::SPHERE_LIST;
	mk.action          = visualization_msgs::Marker::DELETE;
	mk.id              = id;
	pubs_[pub_id].publish(mk);

	mk.action             = visualization_msgs::Marker::ADD;
	mk.pose.orientation.x = 0.0;
	mk.pose.orientation.y = 0.0;
	mk.pose.orientation.z = 0.0;
	mk.pose.orientation.w = 1.0;
    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    while(ros::ok())
    {
        myCodeControlTurtlesim::Person person_msg;
        person_msg.name = "NIUB";
        person_msg.age = 23;
        person_msg.sex = myCodeControlTurtlesim::Person::female;

        person_info_pub.publish(person_msg);
        ROS_INFO("Publish the Person info:name:%s, age:%d, sex:%d",person_msg.name.c_str(),person_msg.age,person_msg.sex);
        
        loop_rate.sleep();

    }

    return 0;
}
