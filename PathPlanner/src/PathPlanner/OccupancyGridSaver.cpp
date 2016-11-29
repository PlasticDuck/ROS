/*
 * OccupancyGridSaver.cpp
 *
 *  Created on: Nov 19, 2016
 *      Author: nandi
 */

#include "OccupancyGridSaver.h"

OccupancyGridSaver::OccupancyGridSaver(nav_msgs::OccupancyGrid occupancyGrid,std::string fileName) {
	this->occupancyGrid=occupancyGrid;
	this->fileName=fileName;
}

OccupancyGridSaver::~OccupancyGridSaver() {
	// TODO Auto-generated destructor stub
}


void OccupancyGridSaver::save(){
	std::ofstream ofs;
	ofs.open(this->fileName.c_str());
	unsigned int row,col;
	row=this->occupancyGrid.info.height;
	col=this->occupancyGrid.info.width;
	ofs<<row<<" "<<col<<std::endl;
	for(unsigned int i=0;i<row;i++){
		for(unsigned int j=0;j<col;j++){
			int val=this->occupancyGrid.data[i*col+j];
			if(val==100){
				ofs<<"#";
			}else{
				ofs<<"_";
			}
		}
		ofs<<std::endl;
	}

	ofs.close();
	ROS_INFO("The map was saved successfully!");

}
