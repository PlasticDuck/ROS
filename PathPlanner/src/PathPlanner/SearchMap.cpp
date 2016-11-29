/*
 * SearchMap.cpp
 *
 *  Created on: Nov 24, 2016
 *      Author: nandi
 */

#include "SearchMap.h"

SearchMap::SearchMap(nav_msgs::OccupancyGrid occupancyGrid) {
	this->width=occupancyGrid.info.width;
	this->height=occupancyGrid.info.height;
	this->map=(int8_t*)malloc(sizeof(int8_t)*width*height);
	this->costMap=std::vector<int8_t>(width*height,0);
	for(int32_t i=0;i<height*width;i++){
		this->map[i]=occupancyGrid.data[i];
		this->costMap[i]=occupancyGrid.data[i];
	}
	this->resolition=occupancyGrid.info.resolution;
	initializeCostMap();
}

SearchMap::~SearchMap() {
	// TODO Auto-generated destructor stub
	free(this->map);
	this->costMap.clear();
}


void SearchMap::initializeCostMap(){
	std::printf("Start initialization the cost map!\n");
	float m_r=0.5;
	float r=m_r/this->resolition;
	for(int32_t y_i=0;y_i<height;y_i++){
		for(int32_t x_i=0;x_i<width;x_i++){
			if(this->costMap[x_i+y_i*width]==100){
				for(int8_t x=-r;x<r;x++){
					for(int8_t y=-r;y<r;y++){
						float val=sqrt(x*x+y*y);
						if(val<=r){
							int8_t cost=(int8_t)((r-val)/r*100);
							//ROS_INFO(" %d",cost);
							if(y_i+y>0 && y_i+y<height && x_i+x>0 && x_i+x<width && cost>this->costMap[(x_i+x)+(y_i+y)*width]){
								this->costMap[(x_i+x)+(y_i+y)*width]=cost;
							}
						}
					}
				}
			}
		}
	}
	std::printf("End initialization the cost map!\n");
}


std::vector<Point2D_i> SearchMap::getNeighbours(Point2D_i point){
	std::vector<Point2D_i> neighbours;
	if(point.x-1>=0){

		Point2D_i p1(point.x-1,point.y);
		neighbours.push_back(p1);
	}
	if(point.x+1<this->width){
		Point2D_i p1(point.x+1,point.y);
		neighbours.push_back(p1);
	}
	if(point.y-1>=0){
		Point2D_i p1(point.x,point.y-1);
		neighbours.push_back(p1);
	}
	if(point.y+1<this->height){
		Point2D_i p1(point.x,point.y+1);

		neighbours.push_back(p1);
	}
	return neighbours;
}

float SearchMap::arcCost(Point2D_i p1, Point2D_i p2){
	//ROS_INFO("From (%d,%d) To (%d,%d)",p1.x,p1.y,p2.x,p2.y);
	if((p1.x==p2.x && (p1.y-1==p2.y || p1.y+1==p2.y)) ||
			(p1.y==p2.y && (p1.x-1==p2.x || p1.x+1==p2.x))){
		float val1=this->costMap[this->Point2Index(p1)];
		float val2=this->costMap[this->Point2Index(p2)];
		if(val1<50 && val2<50){
			//return std::numeric_limits<float>::infinity() ;
			//int cost=val1-val2>0?1:-1;
			//std::printf("%f %f %f,",(float)(val1-val2),val1,val2);
			return abs(val1-val2)+1.0;
		}
		else{
			return std::numeric_limits<float>::infinity() ;
		}

	}
	else if(p1.x==p2.x && p1.y==p2.y){
		return 0;
	}
	return -1;
}

int SearchMap::Point2Index(Point2D_i p){
	return p.x+p.y*this->width;
}

float SearchMap::getResolution(){
	return this->resolition;
}

nav_msgs::OccupancyGrid SearchMap::getOccupancyGrid(){
	nav_msgs::OccupancyGrid grid;
	grid.data.resize(this->costMap.size());
	for(int32_t i=0;i<this->width*this->height;i++){
		grid.data[i]=this->costMap[i];
	}
	return grid;
}

int32_t SearchMap::getSize(){
	return this->height*this->width;
}
