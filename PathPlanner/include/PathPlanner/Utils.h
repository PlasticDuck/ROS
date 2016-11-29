/*
 * Utils.h
 *
 *  Created on: Nov 22, 2016
 *      Author: nandi
 */

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include <boost/container/flat_set.hpp>
#include <boost/tuple/tuple.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <utility>
#include <map>


//This structure will be help to define the states (points)
//,which belongs to the map.
struct Point2D_i_s{
	int32_t x,y;
	Point2D_i_s(int32_t x_,int32_t y_){
		x=x_;
		y=y_;
	};
	Point2D_i_s(){
		x=-1;y=-1;
	}

	bool operator==(const Point2D_i_s p) const{
		return (x==p.x && y==p.y);
	}
//	bool operator!=(const Point2D_i_s p) const{
//		return (x!=p.x || y!=p.y);
//	}
//	bool operator()(const Point2D_i_s p1) const{
//		return (x<p1.x)||(y<p1.y);
//	}

	bool operator<(const Point2D_i_s p1) const{
		return (x<p1.x)||(x==p1.x && y<p1.y);
	}


};

typedef Point2D_i_s Point2D_i;

/*
 * This class can be used to simbolize the points with a float value
 * ,which caraterizes the point.
 */

class PointCost{
	public :
		float cost;
		Point2D_i point;
		PointCost(float cost,Point2D_i point){
			this->cost=cost;
			this->point=point;
		}


		bool operator< (const PointCost p2)const{
			return this->cost<p2.cost;
		}
		bool operator> (const PointCost p2)const{
			return this->cost>p2.cost;
		}
		bool operator()(const PointCost p1,const PointCost p2){
			return p1.cost>p2.cost;
		}
};
/*
 * The object of this class has role to memorize the pairs of point and float value
 * in the ascending order, where the key is the float value.
 */
class PriorityQueueOfPoints{
	boost::container::flat_multiset <PointCost,std::less<PointCost> > queue;

public:
	virtual ~PriorityQueueOfPoints(){
		queue.clear();
	}
	void add(float cost,Point2D_i point){
		PointCost costPoint(cost,point);
		queue.emplace(costPoint);
	}

	PointCost getMin(){
		PointCost pointCost=*queue.begin();
		return pointCost;
	}

	void deleteMin(){
		queue.erase(queue.begin());
	}

	std::pair<boost::container::flat_multiset <PointCost>::iterator,bool> contains(Point2D_i point){
		boost::container::flat_multiset <PointCost>::iterator it=queue.begin();
		boost::container::flat_multiset <PointCost>::iterator end=queue.end();
		std::pair<boost::container::flat_multiset <PointCost>::iterator,bool> result;
		while(it!=end){
			if(it->point.x== point.x && it->point.y==point.y){
				result.first=it;
				result.second=true;
				return result;
			}
			++it;
		}
		result.first=end;
		result.second=false;
		return result;
	}

	bool empty(){
		return queue.empty();
	}
};



#endif /* INCLUDE_UTILS_H_ */
