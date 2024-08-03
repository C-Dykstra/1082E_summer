#include "util.h"
#include "chassis.h"
#include <mutex>
#include <condition_variable>

float calc_dist(Chassis& chassis, Point point){
    Point chassisPoint=chassis.get_coordinates();
    float termOne = chassisPoint.x-point.x;
    float termTwo = chassisPoint.y-point.y;
    return sqrt( termOne*termOne + termTwo*termTwo );
}

float calc_dist(Point pointOne, Point pointTwo){
    float termOne = pointOne.x-pointTwo.x;
    float termTwo = pointOne.y-pointTwo.y;
    return sqrt( termOne*termOne + termTwo*termTwo );
}

Point calcCarrot(Chassis& chassis, Point point, float dLead, float gLead){
    Point chassisPoint=chassis.get_coordinates();
	Point out;
	float dist=calc_dist(chassisPoint, point);
	out.x=chassisPoint.x-dist*sin(chassisPoint.theta)*dLead;
	out.y=chassisPoint.y-dist*cos(chassisPoint.theta)*dLead;

    out.x=point.x*(1-gLead)+out.x*gLead;
    out.y=point.y*(1-gLead)+out.y*gLead;

	return out;
}

class broadcast{
    int instruction=0;
    bool changedFlag=false;

    pros::Mutex mtx;
    std::condition_variable cv;

    public:
    void set_value(int newValue){
        std::lock_guard<pros::Mutex> lock(mtx);
    }
};