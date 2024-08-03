#include <fstream>
#include "logger.h"
#include "motionProfile.h"
#include "pid.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <math.h>
#include <memory>
#include <sstream>
#include <utility>
#include "util.h"
#include <atomic>

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
    float muMG; //mu times mass times g

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

    std::atomic<int> instruction;

    //& Methods

    //~ Odometry

    void odometry(float initialX, float initialY, float initialTheta){

        //. Set Starting Position
        x=initialX;
        y=initialY;
        theta=initialTheta;

        //. Set All Encoder Values to Zero
        horizontalEncoder.reset();
        verticalEncoder.reset();
        inertial.reset();

        //. Main Loop
        while(true){

            //.Finds velocities for calculations
            float leftVelo=leftGroup.get_actual_velocity();
            float rightVelo=rightGroup.get_actual_velocity();
            
            //.Arc Length
            float dist=verticalEncoder.get_position()*M_PI*odometryWheelRadius/18000;
            verticalEncoder.reset_position();
            
            //.Change in Heading
            float deltaTheta=inertialScalar*inertial.get_heading()*M_PI/180.0;
            inertial.reset();

            float radius=dist/deltaTheta; //^ alternate: width*(rightVelo+leftVelo)/(2.0*(rightVelo-leftVelo));

            //.Calculate Change in X & Y
            float intermediate=radius*(sin(deltaTheta)+(1-cos(deltaTheta)));
            float deltaX=cos(theta)*intermediate;
            float deltaY=sin(theta)*intermediate;

            //.Change Position Variables
            x+=deltaX;
            y+=deltaY;
            theta+=deltaTheta;

            //. Delay to Conserve Resources
            pros::delay(10);
        }
    }

    Chassis(pros::MotorGroup& pLeft, pros::MotorGroup& pRight, float pWidth, float pRatio, PID pAng, PID pLat, pros::Rotation pHorizontal, 
    pros::Rotation pVertical, pros::IMU pInertial, float initX, float initY, float initTheta, float pInertialScalar, MotionProfile toFollow, MotionProfile pConstraints, 
    float pOdomWheelRadius=2.75, float pDt=0.01): leftGroup(pLeft), rightGroup(pRight), width(pWidth), ratio(pRatio), angularPID(pAng), 
    lateralPID(pLat), horizontalEncoder(pHorizontal), verticalEncoder(pVertical), inertial(pInertial), x(initX), y(initY), theta(initTheta), inertialScalar(pInertialScalar),
    mainProfile(toFollow), constraints(pConstraints), odometryWheelRadius(pOdomWheelRadius){

        pros::Task odomTask([this]{ odometry(x,y,theta); }); //Starts and runs odometry

        Logger odomLogger("odomLogger", std::make_shared<float>(x),std::make_shared<float>(y)); //Logs position
    }

    public:

    //~ Accessors

    void reverse_chassis(){//TODO Test
        leftGroup.set_reversed_all(true);
        rightGroup.set_reversed_all(true);
        theta+=M_PI;
        theta=fmod(theta,2.0*M_PI);
        reversed=!reversed;
    }

    Point get_coordinates(){
        return Point(x,y,theta);
    }

    std::pair<float,float> get_actual_velocity(){
        return std::make_pair(leftGroup.get_actual_velocity(), rightGroup.get_actual_velocity());
    }

    float get_width(){
        return width;
    }

    MotionProfile get_motion_profile(){
        return mainProfile;
    }

    MotionProfile get_motion_profile_constraints(){
        return constraints;
    }

    float get_muMG(){
        return muMG;
    }

    //~ Chassis Movement Functions
    //# PID Functions

    void move_pid_lat(float dist){
        Point point(x+dist*cos(theta),y+dist*sin(theta));
        float deltaTheta=0;
        while(calc_dist(*this, point)*cos(atan2(point.x-x, point.y-y)-theta)>lateralPID.get_max_error()){
            this->move_velocity(lateralPID.update(dist));
            pros::delay(10);
        }
        lateralPID.reset();
    }

    void boomerang(Point point, float dLead, float gLead=MAXFLOAT, float pSwitchDist=7.5){//TODO Someone else should probably check this method
        if(gLead==MAXFLOAT) gLead=1-dLead;

        lateralPID.toggle_kI(false);
        angularPID.toggle_kI(false);

        if(gLead!=1) {
            boomerang(Point(calcCarrot(*this,point,dLead,gLead),point.theta),dLead,1, pSwitchDist);//TODO! Check max value for endspeed
        }
        else{// If gLead is 1 (Moving to ghost point)

            while(calc_dist(*this, point)<pSwitchDist){
                Point carrot=calcCarrot(*this,point,dLead);

                float angularError=point.theta-theta; //^ Pointing to the right of target is positive error
                float angularVelocity=angularPID.update(angularError)*ratio*wheelRadius; //^ Not actually angular velocity but the linear speed needed to create an angular velocity

                float linearError=calc_dist(*this,point);
                float linearVelocity=lateralPID.update(linearError);

                this->move_velocity(linearVelocity-angularVelocity, linearVelocity+angularVelocity);

                pros::delay(10);
            }
            return;
        }

        while(calc_dist(*this, point)<pSwitchDist){
            Point carrot=calcCarrot(*this,point,dLead);

            float angularError=point.theta-theta;
            float angularVelocity=angularPID.update(angularError)*ratio*wheelRadius;

            float linearError=calc_dist(*this,point);
            float linearVelocity=lateralPID.update(linearError);

            this->move_velocity(linearVelocity-angularVelocity, linearVelocity+angularVelocity);

            pros::delay(10);
        }

        while(calc_dist(*this, point)<lateralPID.get_max_error()){
            Point carrot=calcCarrot(*this,point,dLead);

            float angularError=point.theta-theta;
            float angularVelocity=angularPID.update(angularError)*ratio*wheelRadius;

            float linearError=calc_dist(*this,point)*cos(angularError);
            float linearVelocity=lateralPID.update(linearError);

            this->move_velocity(linearVelocity-angularVelocity, linearVelocity+angularVelocity);

            pros::delay(10);
        }

        this->move_velocity(0);
        lateralPID.toggle_kI(true);
        angularPID.toggle_kI(true);
        return;
    }

    //# Motion Profiling

    void move_mp_lat(float dist, float endSpeed=0){

        Point endPoint=Point(x+dist*cos(theta), y+dist*sin(theta));

        float remainingDist=dist;

        //. Get Initial Speeds
        std::pair<float,float> speeds=this->get_actual_velocity();
        float avgSpeed=(speeds.first+speeds.second)/2.0;

        //. Get Constraints
        float maxSpeed=mainProfile.get_max_speed();
        float maxAccel=mainProfile.get_accel();
        float maxDecel=mainProfile.get_decel();

        //! Assumes that chassis is moving straight if moving
        // The math to correct is very complex if not, involves multiple Euler spirals

        //. Calculate Accel/Decel Distances
        //^ d=v_i*t+a*t^2/2
        float timeToAccel=(maxSpeed-avgSpeed)/maxAccel;
        float accelDist=avgSpeed*timeToAccel+0.5*maxAccel*timeToAccel*timeToAccel;
        remainingDist-=accelDist;

        float timeToDecel=(maxSpeed-endSpeed)/maxDecel;
        float decelDist=maxSpeed*timeToDecel-0.5*maxDecel*timeToDecel*timeToDecel;
        remainingDist-=decelDist;

        float t1,t2,timeAtSpeed;

        if(remainingDist<0){

            // See https://www.desmos.com/calculator/uuptheomfc
            float A=maxAccel*(maxAccel+maxDecel);
            float B=2*avgSpeed*(maxAccel+maxDecel);
            float C=avgSpeed*avgSpeed-2*maxDecel*dist;

            float discriminant=B*B-4*A*C;

            //. If discriminant<0, this method will not be able to work while following constraints, 
            //. so switch to boomerang to ensure it gets there
            if(discriminant<0){
                this->boomerang(Point(endPoint,theta),1);//TODO Tune dLead
                return;
            }

            t1=fmax((-B + std::sqrt(discriminant)) / (2 * A), (-B - std::sqrt(discriminant)) / (2 * A));

            //. Same as above
            if(t1<0){
                this->boomerang(Point(endPoint,theta),1);//TODO Tune dLead
                return;
            }

            t2=(avgSpeed+maxAccel*t1)/maxDecel;

            timeAtSpeed=0;
        }
        else{
            t1=timeToAccel;
            t2=timeToDecel;
            timeAtSpeed=remainingDist/maxSpeed;
        }

        float prevSpeed=avgSpeed;

        while(t1>0){
            float speed=fmin(prevSpeed+maxAccel,maxSpeed);
            this->move_velocity(speed);
            prevSpeed=speed;
            t1-=.01;
            pros::delay(10);
        }

        while(timeAtSpeed>0){
            this->move_velocity(maxSpeed);
            timeAtSpeed-=.01;
            pros::delay(10);
        }

        while(t2>0){
            float speed=fmax(prevSpeed-maxDecel,endSpeed);
            this->move_velocity(speed);
            prevSpeed=speed;
            t1-=.01;
            pros::delay(10);
        }
        return;
    }

    void move_2d_mp(std::string fileName){

        std::ifstream file(fileName + ".txt");
        if (!file) {
        std::cerr << "Error opening file: " << fileName << ".txt" << std::endl;
        return;
        }

        std::string line;
        int i=0;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            if(line.starts_with('I')){
                int next;
                ss>>next;
                instruction=next;
            }
            else{
                //Format:
                //x,y,left,right
                //instruction
                float discard,left,right;
                ss>>discard>>discard>>left>>right;
                this->move_velocity(left,right);
                pros::delay(10);
            }
        }
        file.close();     
    }

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