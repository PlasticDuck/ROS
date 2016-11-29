/*
 * DStarPlanner.h
 *
 *  Created on: Nov 23, 2016
 *      Author: nandi
 *
 *	The object of this class has role to find the optimal path
 *	from the start state to the goal state.
 *	The start state and the goal state are memorized in the instances of Point2D_i
 *	structure, which are the inputs of constructor. The search space is described by
 *	OccupancyGrid, of which the data symbolizes search space structure. Height and width describe
 *	the dimension of the map.
 */

#ifndef DSTARPLANNER_H_
#define DSTARPLANNER_H_

#include "Utils.h"
#include "SearchMap.h"

#include <map>
#include <functional>
#include <vector>
#include <limits>
#include <stdio.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>


namespace PathPlanner {

class DStarPlanner {
public:
	DStarPlanner(nav_msgs::OccupancyGrid,Point2D_i,Point2D_i);
	DStarPlanner(SearchMap*,Point2D_i,Point2D_i);
	virtual ~DStarPlanner();
	int pathprocess();
	void cicle();
	nav_msgs::Path getPath();
private:
//	Problem describtors (parameters)
	SearchMap *searchmap;
	Point2D_i start,goal;

//	Problem rezolvers (parameters)
	PriorityQueueOfPoints queue;//OPEN LIST
	std::map<Point2D_i,Point2D_i,std::less<Point2D_i> > backpointers;

//	true=OPEN, false=CLOSED
	std::map<Point2D_i,bool> t;//State of points
//	Memoziring the path's cost from the one point to the goal point
	std::vector<float> h;
// Private Methods
	void initialize();
	float SumOfPath(Point2D_i);
	void insertLow(Point2D_i,Point2D_i,float);
	void insertLower(Point2D_i,Point2D_i,float,float);
	void insertRaised(Point2D_i,Point2D_i,float,float);

	void insert(Point2D_i,float);
	void viewPath();
};


/*
 * The object of this class memorize the backpointer pairs
 * in the list. A backpointer pair is created from two states (Point),
 * where the first state X is the starting point and the second state Y
 * is the end point.
 */


} /* namespace Planner */




#endif /* DSTARTPLANNER_H_ */
