/*
 * TransformatorOOG.cpp
 *
 *  Created on: Nov 19, 2016
 *      Author: nandi
 */

#include "TransformatorOOG.h"

TransformatorO_OG::TransformatorO_OG(nav_msgs::MapMetaData mapMetaData,tf::TransformListener *listener,ros::Time begin) {
//	MapResolution simbolize the connection between the map and real world [cell/m]
	this->mapResolution=1.0/mapMetaData.resolution;
	this->listener=listener;
	this->begin=begin;
}

TransformatorO_OG::~TransformatorO_OG() {
	// TODO Auto-generated destructor stub
}


Point2D_i TransformatorO_OG::transform(nav_msgs::Odometry odom){
	std::string error;
//	Verify the transformation executability
	if(!listener->canTransform("map","odom",ros::Time::now(),&error)){
		ROS_INFO("!The translation from map to odom isn't executable! %s",error.c_str());
		Point2D_i mapCoor;
		mapCoor.x=-1;mapCoor.y=-1;
		return mapCoor;
	}
//	Initialize the position and oriantation of the robot
	tf::Pose currentPose;tf::Pose transPose;
//	Transfrom the pose from geometry_msgs to tf
	tf::poseMsgToTF(odom.pose.pose,currentPose);
// 	Transform between frames

//	Creating a PointStamped, which corresponds the point on the source frame
//	Initializing the posion and header
	geometry_msgs::PointStamped pointStamped,transPoint;
	pointStamped.point=odom.pose.pose.position;
	pointStamped.header=odom.header;
//	Transforming and memorizing in "transPoint the point"
	this->listener->transformPoint("map",pointStamped,transPoint);
//	ROS_INFO("Transfrom result: %f %f %f",transPoint.point.x,transPoint.point.y,transPoint.point.z);
//
//	Calculation the robot's position on the map
	double x_map,y_map;
	x_map=transPoint.point.x*this->mapResolution;
	y_map=transPoint.point.y*this->mapResolution;
//	ROS_INFO("Map coor: %f %f, Resolution: %f",x_map,y_map,this->mapResolution);
//	It will convert the x_map,y_map to the specified structure
	Point2D_i mapCoor;
	mapCoor.x=(int)x_map;
	mapCoor.y=(int)y_map;
	return mapCoor;
}

