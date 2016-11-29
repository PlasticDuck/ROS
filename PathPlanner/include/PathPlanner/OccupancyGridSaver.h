/*
 * OccupancyGridSaver.h
 *
 *  Created on: Nov 19, 2016
 *      Author: nandi
 */

#ifndef OCCUPANCYGRIDSAVER_H_
#define OCCUPANCYGRIDSAVER_H_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <fstream>
#include <string>



class OccupancyGridSaver {
public:
	OccupancyGridSaver(nav_msgs::OccupancyGrid,std::string);
	virtual ~OccupancyGridSaver();
	void save();
private:
	nav_msgs::OccupancyGrid occupancyGrid;
	std::string fileName;
};

#endif /* OCCUPANCYGRIDSAVER_H_ */
