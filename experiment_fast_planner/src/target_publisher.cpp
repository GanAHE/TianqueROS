#include "experiment_fast_planner/target_publisher.h"


int main(int argc,char **argv)
{
    ros::init(argc,argv,"target_publisher");

    ros::NodeHandle n;

    ros::Publisher person_info_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);

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
        geometry_msgs::geometry_msgs person_msg;
        person_msg.orientation.x = 1.0;

        person_info_pub.publish(person_msg);
        ROS_INFO("Publish the Person info:name:%s, age:%d, sex:%d",person_msg.name.c_str(),person_msg.age,person_msg.sex);
        
        loop_rate.sleep();

    }

    return 0;
}
