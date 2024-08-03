#pragma once
#include <fstream>
#include "motionProfile.h"
#include "pid.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <math.h>
#include <sstream>
#include <utility>
#include "util.h"

class Chassis{

    //& Variables

    //~ Motors

    pros::MotorGroup& leftGroup;
    pros::MotorGroup& rightGroup;

    bool reversed=false;

    //~ Chassis Info

    float width;
    float ratio;
    float wheelRadius;

    //~ PID

    PID angularPID;
    PID lateralPID;

    //~ Odometry

    pros::Rotation horizontalEncoder;
    pros::Rotation verticalEncoder;
    pros::IMU inertial;

    float odometryWheelRadius=2.75;

    double x;
    double y;
    double theta; //Standard (0 radians to right)

    float inertialScalar=1;

    //~ Motion Profiles

    MotionProfile mainProfile;
    MotionProfile constraints;

    //~ Other
    std::vector<int> instructions; //^ Used to activate subsystems mid-path
    //TODO Replace

    //& Methods

    //~ Odometry

    void odometry(float initialX, float initialY, float initialTheta);

    Chassis(pros::MotorGroup& pLeft, pros::MotorGroup& pRight, float pWidth, float pRatio, PID pAng, PID pLat, pros::Rotation pHorizontal, 
    pros::Rotation pVertical, pros::IMU pInertial, float initX, float initY, float initTheta, float pInertialScalar, MotionProfile toFollow, MotionProfile pConstraints, 
    float pOdomWheelRadius=2.75, float pDt=0.01): leftGroup(pLeft), rightGroup(pRight), width(pWidth), ratio(pRatio), angularPID(pAng), 
    lateralPID(pLat), horizontalEncoder(pHorizontal), verticalEncoder(pVertical), inertial(pInertial), x(initX), y(initY), theta(initTheta), inertialScalar(pInertialScalar),
    mainProfile(toFollow), constraints(pConstraints), odometryWheelRadius(pOdomWheelRadius){}

    public:

    //~ Accessors

    void reverse_chassis();

    Point get_coordinates();

    std::pair<float,float> get_actual_velocity();

    float get_width();

    MotionProfile get_motion_profile();

    MotionProfile get_motion_profile_constraints();

    float get_muMg();

    //~ Chassis Movement Functions
    //# PID Functions

    void move_pid_lat(float dist);

    void boomerang(Point point, float dLead, float gLead=MAXFLOAT, float pSwitchDist=7.5);

    //# Motion Profiling

    void move_mp_lat(float dist, float endSpeed=0);

    void move_2d_mp(std::vector<Point> path);

    private:

    std::vector<Point> parse_2DMP_path(std::string fileName){

        std::vector<Point> out;

        std::ifstream file(fileName + "_MPPath.txt");
        if (!file) {
            std::cerr << "Error opening file: " << fileName << ".txt" << std::endl;
            return out;
        }

        std::string line;
        while(std::getline(file, line)){
            Point newPoint;
            if(line.substr(0,5)!="INSTR"){
                std::stringstream lineOut(line);
                lineOut>>newPoint.x>>newPoint.y>>newPoint.leftVelo>>newPoint.rightVelo;
                out.emplace_back(newPoint);
            }
            else{
                //TODO Impl
                std::stringstream lineOut(line);
                lineOut>>out[out.size()-1].instruction;
            }
        }
        return out;
    }

    public:

    //# General Movement Functions

    void move_velocity(float velocity){
        leftGroup.move_velocity(velocity);
        rightGroup.move_velocity(velocity);
    }
    void move_velocity(float velocityLeft, float velocityRight){ //TODO May want a self-tuned velo controller
        if(!reversed){
            leftGroup.move_velocity(velocityLeft);
            rightGroup.move_velocity(velocityRight);
        }
        else{
            leftGroup.move_velocity(velocityRight);
            rightGroup.move_velocity(velocityLeft);
        }
    }

};