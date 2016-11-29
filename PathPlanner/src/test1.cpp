#include <ros/ros.h>


#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <boost/tuple/tuple.hpp>

#include "OccupancyGridSaver.h"
#include "TransformatorOOG.h"
#include "Utils.h"
#include "DStarPlanner.h"

//Persistance include


using namespace ros;

nav_msgs::Odometry initialPosition;
bool isInitPosition=false;

void nav_callback(const nav_msgs::OdometryConstPtr odom){
	initialPosition=(*odom);
	isInitPosition=true;
}

int main(int argc, char **argv) {
	// Initialize the node
	init(argc, argv, "test_map_server");

	nav_msgs::OccupancyGrid proba;
	// Create a node handle
	NodeHandle node;

	//Initialize begin time and timeOut duration
	ros::Time begin=ros::Time::now();
	ros::Duration timeOut=ros::Duration(10);
//	Initialize the transformlistener
//	cache time will be 50sec
//	spin thead will be true
	tf::TransformListener listener(ros::Duration(50),true);
	listener.waitForTransform("map","odom",begin,timeOut);

//	It will be sleeping during the predefined timeOut,
//	it will be help to be initialized the transform listener
	ROS_INFO("It's sleeping!");
	timeOut.sleep();
	ROS_INFO("It waked up!");

//	This subscriber help to memorize the robot's initial position from the frame odom
	Subscriber sub = node.subscribe("odom", 1000, nav_callback);

//	Receiving the static grid map of the plane. It will be used for the path planning.
	ros::ServiceClient client=node.serviceClient<nav_msgs::GetMap>("static_map");
	nav_msgs::GetMap srv_map;
	nav_msgs::OccupancyGrid occupancyGrid;
	if(client.call(srv_map)){
//	Succesfully receiving the static grid map and memorizing in the variable!
	ROS_INFO("Received succesfully the static_map!");
	occupancyGrid=srv_map.response.map;
//	ROS_INFO("W:%d H:%d",occupancyGrid.info.width,occupancyGrid.info.height);
	}else{
		ROS_INFO("Map server hasn't started yet! Please run the launcher file named \"test\".");
		return 1;
	}

//	Receiving the initial position of the robot in the plane.
	while(!isInitPosition){
//		  ROS_INFO("%s",listener.allFramesAsString().c_str());
	  ros::spinOnce();
	}

//	Test the executability of transfrom
	std::string error;
	if(!listener.canTransform("map","odom",begin,&error)){
		ROS_INFO("The translation from map to odom isn't executable! %s",error.c_str());
	}
//	This transformation is used to transform the robot's position from the frame odom
//	to the frame map.
	TransformatorO_OG transformator(occupancyGrid.info,&listener,begin);
	Point2D_i init_RobotMapCoor=transformator.transform(initialPosition);
	Point2D_i goal(34,181);
	ROS_INFO("(%d,%d) (%d,%d)",init_RobotMapCoor.x,init_RobotMapCoor.y,goal.x,goal.y);

	SearchMap map(occupancyGrid);
	PathPlanner::DStarPlanner planner(&map,init_RobotMapCoor,goal);
	planner.cicle();

//	Creating a publisher with visialisation_msgs about path
	ros::Publisher pathPub=node.advertise<nav_msgs::Path>("my_path",100);
	ros::Publisher mapPub=node.advertise<nav_msgs::OccupancyGrid>("costMap",100);
	nav_msgs::OccupancyGrid occupancyGrid_costMap=map.getOccupancyGrid();
	occupancyGrid_costMap.info=occupancyGrid.info;

	nav_msgs::Path trai=planner.getPath();
	occupancyGrid_costMap.header=trai.header;
	ros::Rate r(0.5);
	int seq=0;
	while(ros::ok()){
		//ROS_INFO("Publish mypath and costMap!");
		trai.header.seq=seq;
		occupancyGrid_costMap.header.seq=seq;
//		seq++;
		mapPub.publish(occupancyGrid_costMap);
		pathPub.publish(trai);
		r.sleep();
	}


	ROS_INFO("Finished a run!");
	return 0;
}
