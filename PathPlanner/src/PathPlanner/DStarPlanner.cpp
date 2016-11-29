/*
 * DStartPlanner.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: nandi
 */

#include "DStarPlanner.h"

namespace PathPlanner {

DStarPlanner::DStarPlanner(nav_msgs::OccupancyGrid occupancyGrid,Point2D_i start,Point2D_i goal) {
	// TODO Auto-generated constructor stub
	this->searchmap=new SearchMap(occupancyGrid);
	this->start=start;
	this->goal=goal;
	this->h=std::vector<float>(this->searchmap->getSize(),std::numeric_limits<float>::infinity());
	initialize();
}

DStarPlanner::DStarPlanner(SearchMap* map,Point2D_i start,Point2D_i goal){
	this->searchmap=map;
	this->start=start;
	this->goal=goal;
	this->h=std::vector<float>(this->searchmap->getSize(),std::numeric_limits<float>::infinity());
	initialize();
}



DStarPlanner::~DStarPlanner() {
	// TODO Auto-generated destructor stub
	this->h.clear();
	this->t.clear();
	this->backpointers.clear();
}

void DStarPlanner::cicle(){
	std::printf("It started to find the path!\n");
	int i=0;
	int state=0;
	while(state!=1){
		state=pathprocess();
		if(state==-1){
			ROS_INFO("DStarPlanner didn't found a path to the goal!");
			return;
		}
		i++;
		if((i*100)%(this->searchmap->getSize()*10)==0){
			ROS_INFO("%2.2f[%%]",((float)i/this->searchmap->getSize()*100.0));
			//std::printf("%f ",((float)i/this->searchmap->getSize()*100.0));
		}
	}
	ROS_INFO("DStarPlanner found a path to the goal!");
}

int DStarPlanner::pathprocess(){
	if(this->queue.empty()){
		return -1;
	}
	PointCost minPointCost=this->queue.getMin();
	float k_old=minPointCost.cost;
	Point2D_i minState=minPointCost.point;
	if(minState==this->start){
		return 1;
	}

	this->t[minState]=false;
	this->queue.deleteMin();
//	It represents the H(X) value
	float cost=this->h[this->searchmap->Point2Index(minState)];//this->SumOfPath(minState);
//	ROS_INFO("State:(%d,%d)  K_old:%f Cost:%f",minState.x,minState.y,k_old,cost);

//	float cost_h=this->SumOfPath(minState);
//	if(cost!=cost_h){
//		ROS_INFO("Costs aren't same. H:%2.2f Func:%2.2f",cost,cost_h);
//	}

	if(k_old<cost){
//		It will necesarry to process the neighbours states.
		ROS_INFO("Raised");
		std::vector<Point2D_i> neighbours=this->searchmap->getNeighbours(minState);
		std::vector<Point2D_i>::iterator it=neighbours.begin();
		int i=0;
		while(it!=neighbours.end()){
			//ROS_INFO("Neighbour %d:(%d,%d)",i,it->x,it->y);
			this->insertRaised(*it,minState,cost,k_old);
			i++;
			++it;
		}
	}
	else if(k_old==cost){
//		It will necesarry to process the neighbours states.
//		ROS_INFO("Equal");
		std::vector<Point2D_i> neighbours=this->searchmap->getNeighbours(minState);
		std::vector<Point2D_i>::iterator it=neighbours.begin();
		int i=0;
		while(it!=neighbours.end()){
//			Test the state (point) is NEW
			this->insertLow(*it,minState,cost);
			i++;
			++it;
		}
	}
	else{
//		It will necesarry to process the neighbours states.
		ROS_INFO("Lower");
		std::printf("Point(%d,%d) key:%f cost:%f ",minState.x,minState.y,k_old,cost);
		std::vector<Point2D_i> neighbours=this->searchmap->getNeighbours(minState);
		std::vector<Point2D_i>::iterator it=neighbours.begin();
		int i=0;
		while(it!=neighbours.end()){
//			this->insertLow(*it,minState,cost,1);
//			ROS_INFO("Neighbour %d:(%d,%d)",i,it->x,it->y);
			this->insertLower(*it,minState,cost,k_old);
			i++;
			++it;
		}

	}
	return 0;
}

void DStarPlanner::initialize(){
	this->queue.add(0.0,this->goal);
	this->t[this->goal]=true;
	this->backpointers[this->goal]=goal;
	this->h[this->searchmap->Point2Index(this->goal)]=0;

}
/*
 * It's caluclate the cost of the path from the input point to the goal.
 * This function using the backpointers, if it found a point, which doesn't exist in the
 * backpointers, than returns inf.
 */
float DStarPlanner::SumOfPath(Point2D_i point){
	float sum=0;
	std::map<Point2D_i,Point2D_i,std::less<Point2D_i> >::iterator it=this->backpointers.find(point);
	if(this->backpointers.count(point)==0){
		viewPath();
		return std::numeric_limits<float>::infinity();
	}

	Point2D_i endPoint(-1,-1);
	Point2D_i startPoint=point;
	while(!(startPoint==this->goal)){
		if(this->backpointers.count(startPoint)==0){
			return std::numeric_limits<float>::infinity();
		}
		it=this->backpointers.find(startPoint);
		endPoint=it->second;
		sum+=this->searchmap->arcCost(startPoint,endPoint);
		startPoint=endPoint;
	}
	return sum;
}

/*
 *	This function is used in the case, when the state's key equals with the state's cost.
 */
void DStarPlanner::insertLow(Point2D_i newPoint,Point2D_i minPoint,float minCost){
/*
 * 	1th-cond: it's a new state in the OPEN list
 * 	2th-cond: the state exists in the OPEN but the path length doesn't equal with relity.
 * 	3th-cond: the state exists in the OPEN but throught another point ,which results a longer path
 */
	if(this->t.count(newPoint)==0 ||
			(this->backpointers[newPoint]==minPoint && /*this->SumOfPath(newPoint)*/
					this->h[this->searchmap->Point2Index(newPoint)]!=minCost+this->searchmap->arcCost(newPoint,minPoint)) ||
			(!(this->backpointers[newPoint]==minPoint) && /*this->SumOfPath(newPoint)*/
					this->h[this->searchmap->Point2Index(newPoint)]>minCost+this->searchmap->arcCost(newPoint,minPoint))){
		this->backpointers[newPoint]=minPoint;
		float costNeighbour=minCost+this->searchmap->arcCost(newPoint,minPoint);
		this->h[this->searchmap->Point2Index(newPoint)]=costNeighbour;
	/*	if(this->t.count(newPoint)==0){
			ROS_INFO("Added to OPEN a NEW STATE(%d,%d) with %f",newPoint.x,newPoint.y,costNeighbour);
		}else{
			if(this->backpointers[newPoint]==minPoint){//OPEN
				ROS_INFO("Update the key of STATE(%d,%d) with %f",newPoint.x,newPoint.y,costNeighbour);
			}
			else{
				ROS_INFO("Change the backpointer and key of STATE(%d,%d) with %f",newPoint.x,newPoint.y,costNeighbour);
			}
		}
*/		this->insert(newPoint,costNeighbour);
	}
}

void DStarPlanner::insertLower(Point2D_i newPoint,Point2D_i minPoint,float minCost,float k_old){
	if(this->t.count(newPoint)==0 ||
		(this->backpointers[newPoint]==minPoint && /*this->SumOfPath(newPoint)*/
				this->h[this->searchmap->Point2Index(newPoint)] !=minCost+this->searchmap->arcCost(newPoint,minPoint))){
		this->backpointers[newPoint]=minPoint;
		float costNeighbour=minCost+this->searchmap->arcCost(newPoint,minPoint);
		this->h[this->searchmap->Point2Index(newPoint)]=costNeighbour;
		this->insert(newPoint,costNeighbour);
	}else if(!(this->backpointers[newPoint]==minPoint) && /*this->SumOfPath(newPoint)*/
			this->h[this->searchmap->Point2Index(newPoint)]>minCost+this->searchmap->arcCost(minPoint,newPoint)){
		this->insert(minPoint,minCost);
	}else if(!(this->backpointers[newPoint]==minPoint) && /*this->SumOfPath(newPoint)*/
			this->h[this->searchmap->Point2Index(newPoint)]>minCost+this->searchmap->arcCost(minPoint,newPoint) &&
			!this->t[newPoint] && /*this->SumOfPath(newPoint)*/
			this->h[this->searchmap->Point2Index(newPoint)]>k_old){
		this->insert(newPoint,this->SumOfPath(newPoint));
	}
}

void DStarPlanner::insertRaised(Point2D_i newPoint,Point2D_i minPoint,float cost,float keyOld){
	float h_y=this->h[this->searchmap->Point2Index(newPoint)];//this->SumOfPath(newPoint);
	if(this->t.count(newPoint)>0 && h_y<keyOld && cost>h_y+this->searchmap->arcCost(minPoint,newPoint)){
		this->backpointers[minPoint]=newPoint;
		this->h[this->searchmap->Point2Index(minPoint)]=h_y+this->searchmap->arcCost(minPoint,newPoint);
	}
}

void DStarPlanner::viewPath(){
	std::map<Point2D_i,Point2D_i,std::less<Point2D_i> >::iterator it=backpointers.begin();
	while(it!=backpointers.end()){
		ROS_INFO("(%d,%d) (%d,%d)",it->first.x,it->first.y,it->second.x,it->second.y);
		++it;
	}
}



void DStarPlanner::insert(Point2D_i point,float key){
//	It searches in the priority queue the point
	std::pair<boost::container::flat_multiset <PointCost>::iterator,bool> result=this->queue.contains(point);
	if(this->queue.contains(point).second){
//		If it exists, than point's key is necessary to be updated.
		result.first->cost=key;
	}
	else{
		this->queue.add(key,point);
		this->t[point]=true;
	}
}

nav_msgs::Path DStarPlanner::getPath(){
	nav_msgs::Path myTraiectory;
	std::vector<geometry_msgs::PoseStamped> plan;
	ros::Time justNow=ros::Time::now();
	Point2D_i i=this->start;
	while(!(i==this->goal)){
		if(this->backpointers.count(i)==0){
			break;
		}
		Point2D_i nextPoint=this->backpointers[i];
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x=((float)i.x+0.5)*this->searchmap->getResolution();
		pose.pose.position.y=((float)i.y+0.5)*this->searchmap->getResolution();
		pose.pose.position.z=0;
		pose.pose.orientation.x=pose.pose.orientation.y=pose.pose.orientation.z=0;
		pose.pose.orientation.w=1;
		pose.header.frame_id="map";
		pose.header.stamp=justNow;
		plan.push_back(pose);

		i=nextPoint;
	}

	if(plan.size()>0){
		myTraiectory.header=plan[0].header;
		myTraiectory.poses.resize(plan.size());
		for(int j=0;j<plan.size();j++){
			myTraiectory.poses[j]=plan[j];
		}
	}
	return myTraiectory;
}

} /* namespace Planner */
