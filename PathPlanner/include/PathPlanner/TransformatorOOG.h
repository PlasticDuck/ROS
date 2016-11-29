/*
 * TransformatorOOG.h
 *
 *  Created on: Nov 19, 2016
 *      Author: nandi
 *
 *  The object of this class can transform the position of robot,
 *  which is represented with Odometry structure. For transformation is neccessary
 *  some information about the OccupancyGrid, those informations are memorized in the MapMetaData
 *  structure, which will be the input of the constructure.
 */

#ifndef TRANSFORMATOROOG_H_
#define TRANSFORMATOROOG_H_

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "Utils.h"

class TransformatorO_OG {
public:
	TransformatorO_OG(nav_msgs::MapMetaData,tf::TransformListener*,ros::Time);
	virtual ~TransformatorO_OG();
	Point2D_i transform(nav_msgs::Odometry);
private:
//	MapResolution [cell/m]
	double mapResolution;
	tf::TransformListener *listener;
	ros::Time begin;
};

#endif /* TRANSFORMATOROOG_H_ */
