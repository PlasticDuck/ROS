/*
 * SearchMap.h
 *
 *  Created on: Nov 24, 2016
 *      Author: nandi
 */

#ifndef SEARCHMAP_H_
#define SEARCHMAP_H_

#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <ros/ros.h>
#include <limits>

#include "Utils.h"
#include <stdio.h>

class SearchMap {
	int32_t width,height;
	int8_t *map;
	std::vector<int8_t> costMap;
	float resolition;
public:
	SearchMap(nav_msgs::OccupancyGrid);
	virtual ~SearchMap();
	std::vector<Point2D_i> getNeighbours(Point2D_i);
	float arcCost(Point2D_i p1,Point2D_i p2);
	int Point2Index(Point2D_i);
	float getResolution();
	nav_msgs::OccupancyGrid getOccupancyGrid();
	int32_t getSize();
private:
	void initializeCostMap();
};

#endif /* SEARCHMAP_H_ */
