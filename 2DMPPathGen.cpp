#include "motionProfile.h" 
#include <climits>
#include <ostream>
#include <vector>
#include <cmath>
#include "chassis.h"
#include "util.h"

static bool get_linearity(Point p1, Point p2, Point p3){
    return (p2.y - p1.y) * (p3.x - p2.x) == (p3.y - p2.y) * (p2.x - p1.x);
}

static float get_curvature(Point p1, Point p2, Point p3){

    if(get_linearity(p1,p2,p3)) return 0;

    float crossProduct=(p2.x-p1.x)*(p3.y-p2.y)-(p2.y-p1.y)*(p3.x-p2.x);
    float sideLength1=calc_dist(p1,p2);
    float sideLength2=calc_dist(p2, p3);
    float sideLength3=calc_dist(p3,p1);

    return crossProduct/(sideLength1*sideLength2*sideLength3);//< If positive, curves left, if negative, curves right
}

/*
static void MPPathGen(std::string fileName, std::vector<Point> path, MotionProfile profile, Chassis chassis) {
    
    std::ifstream file(fileName + ".txt");
    if (!file) {
        std::cerr << "Error opening file: " << fileName << ".txt" << std::endl;
        return;
    }

    std::string line;
    std::vector<Point> points;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        float x, y;
        ss >> x >> y;
        points.emplace_back(x, y);
    }
    file.close();

    std::ofstream outLine(fileName + "_MPPath.txt");
    if (!outLine) {
        std::cerr << "Error opening output file: " << fileName + "_MPPath.txt" << std::endl;
        return;
    }

    float maxSpeed = profile.get_max_speed();
    float leftSpeed;
    float rightSpeed;

    std::vector<Point> midway;

    float leftPrevSpeed = 0;
    float rightPrevSpeed = 0;

    float maxAccel = profile.get_accel();
    float maxDecel = profile.get_decel();

    for(int i=0;i<points.size()-2;i++){
        float curvature=get_curvature(points[i], points[i + 1], points[i + 2]);
        float lesser=maxSpeed*(1-chassis.get_width()*fabs(curvature));

        if(curvature<0){
            leftSpeed=maxSpeed;
            rightSpeed=lesser;
        }
        else{
            leftSpeed=lesser;
            rightSpeed=maxSpeed;
        }

        float leftAccel=fmin(leftPrevSpeed+maxAccel*.01,(curvature==0)?INT_MAX:sqrt(profile.get_mu()*profile.get_weight()/fabs(curvature)));
        if(leftSpeed>leftAccel||maxSpeed){
            float ratio=fmin(leftAccel,maxSpeed)/leftSpeed;
            leftSpeed*=ratio;
            rightSpeed*=ratio;
        }
        float rightAccel=fmin(rightPrevSpeed+maxAccel*.01,(curvature==0)?INT_MAX:sqrt(profile.get_mu()*profile.get_weight()/fabs(curvature)));
        if(rightSpeed>rightAccel||maxSpeed){
            float ratio=fmin(rightAccel,maxSpeed)/rightSpeed;
            leftSpeed*=ratio;
            rightSpeed*=ratio;
        }

        leftPrevSpeed=leftSpeed;
        rightPrevSpeed=rightSpeed;

        midway.emplace_back(points[i],leftSpeed,rightSpeed);
    }

    midway.emplace_back(points.back(), 0, 0);
    if (points.size() > 1) midway.emplace_back(points[points.size() - 2], 0, 0);

    leftPrevSpeed=0;
    rightPrevSpeed=0;

    for (int i = points.size() - 3; i > 0; i--) {

        float leftSpeed=midway.at(i).leftVelo;
        float rightSpeed=midway.at(i).rightVelo;

        float leftAccel=leftPrevSpeed+maxDecel*.01;
        if(leftSpeed>leftAccel){//< TODO MAX DECEL IS POSITIVE!!!!!!!
            float ratio=leftSpeed/leftAccel;
            leftSpeed*=ratio;
            rightSpeed*=ratio;
        }
        float rightAccel=rightPrevSpeed+maxDecel*.01;
        if(rightSpeed>rightAccel){
            float ratio=rightSpeed/rightAccel;
            leftSpeed*=ratio;
            rightSpeed*=ratio;
        }

        leftPrevSpeed=leftSpeed;
        rightPrevSpeed=rightSpeed;

        outLine << midway[i].x << " " << midway[i].y << " " << leftSpeed << " " << rightSpeed << std::endl;
    }

    outLine.close();
}
*/

static Point interpolate(Point start, Point end, float ratio){
    Point out;

    out.x=start.x+ratio*(end.x-start.x);
    out.y=start.y+ratio*(end.y-start.y);
    out.theta=start.theta+ratio*(end.theta-start.theta);

    out.leftVelo=start.leftVelo+ratio*(end.leftVelo-start.leftVelo);
    out.rightVelo=start.rightVelo+ratio*(end.rightVelo-start.rightVelo);

    out.timeToPoint=start.timeToPoint+0.01;

    out.instruction=end.instruction;

    return out;
}

float calc_dist(Point start, Point end, float curvature, Chassis chassis){
    float dist=calc_dist(start,end);
    if(curvature!=0) dist=2*asin(dist*curvature/2.0)*(1/curvature-chassis.get_width()/2.0);
    return dist;
}

void mp_path_gen(std::string fileName, Chassis chassis){
    std::vector<Point> waypoints;

    //. File Reading and Output Setup
    std::ifstream file(fileName + ".txt");
    if (!file) {
        std::cerr << "Error opening file: " << fileName << ".txt" << std::endl;
        return;
    }

    std::string line;
    int i=0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        if(line=="endData") break;
        if(line.starts_with('I')){
            if(i!=0) ss>>waypoints[i-1].instruction;
        }
        else{
            float x, y;
            ss >> x >> y;
            waypoints.emplace_back(x, y);
        }
        i++;
    }
    file.close();

    std::ofstream outLine(fileName + "_MPPath.txt");
    if (!outLine) {
        std::cerr << "Error opening output file: " << fileName + "_MPPath.txt" << std::endl;
        return;
    }
    //.

    MotionProfile profile=chassis.get_motion_profile();

    float maxSpeed = profile.get_max_speed();
    float maxAccel = profile.get_accel();
    float maxDecel = profile.get_decel();

    waypoints[0].timeToPoint=0;

    for(int i=0;i<waypoints.size()-1;i++){
        Point start=waypoints[i];
        Point end=waypoints[i+1];

        float curvature;

        if(i==waypoints.size()-1){ curvature=get_curvature(waypoints[i-1], start, end); }
        else { curvature=get_curvature(start, end, waypoints[i+2]); }

        if(curvature==0) curvature+=0.000001;

        float leftPrevSpeed=((i==0)?0:waypoints[i-1].leftVelo)/2.0;
        float rightPrevSpeed=((i==0)?0:waypoints[i-1].rightVelo)/2.0;

        //^ Accel = min ( curvature limited, max speed limited, max accel limited)
        float leftAccel=fmin((sqrt(chassis.get_muMg()* (1/fabs(curvature)-chassis.get_width()/2.0))-leftPrevSpeed)/2.0, fmin(maxAccel,(maxSpeed-leftPrevSpeed)/2.0 ));
        float rightAccel=fmin((sqrt(chassis.get_muMg()* (1/fabs(curvature)-chassis.get_width()/2.0))-rightPrevSpeed)/2.0, fmin(maxAccel,(maxSpeed- rightPrevSpeed)/2.0 ));

        float leftSpeed;
        float rightSpeed;

        float lesser=maxSpeed*(1-chassis.get_width()*fabs(curvature));

        if(curvature<0){
            leftSpeed=maxSpeed;
            rightSpeed=lesser;
        }
        else{
            leftSpeed=lesser;
            rightSpeed=maxSpeed;
        }

        if(leftSpeed>leftAccel||maxSpeed){
            float ratio=fmin(leftAccel,maxSpeed)/leftSpeed;
            leftSpeed*=ratio;
            rightSpeed*=ratio;
        }

        if(rightSpeed>rightAccel||maxSpeed){
            float ratio=fmin(rightAccel,maxSpeed)/rightSpeed;
            leftSpeed*=ratio;
            rightSpeed*=ratio;
        }

        start.leftVelo=leftSpeed;
        start.rightVelo=rightSpeed;        
    }

    outLine<<waypoints[0].x<<" "<<waypoints[0].y<<" "<<waypoints[0].leftVelo<<" "<<waypoints[0].rightVelo<<std::endl;
    if(waypoints[0].instruction!=0) outLine<<waypoints[0].instruction<<std::endl;

    for(int i=0;i<waypoints.size()-1;i++){
        Point start=waypoints[i];
        Point end=waypoints[i+1];

        float curvature, dist;

        //^ Calculates curvature
        if(i==waypoints.size()-1){ curvature=get_curvature(waypoints[i-1], start, end); }
        else { curvature=get_curvature(start, end, waypoints[i+2]); }

        if(curvature==0) curvature+=0.000001; //^ Prevents div by 0 error

        //^ Calculates arc distance between points in order to find the correct ratio
        dist=calc_dist(start,end,curvature,chassis);

        float deltaTime=dist/fmin(start.leftVelo,start.rightVelo);

        Point newPoint=interpolate(start,end,.01/deltaTime);

        waypoints[i+1]=newPoint;

        outLine<<newPoint.x<<" "<<newPoint.y<<" "<<newPoint.leftVelo<<" "<<newPoint.rightVelo<<std::endl;
        if(newPoint.instruction!=0) outLine<<newPoint.instruction<<std::endl;
    }

    Point start=waypoints[waypoints.size()-2];
    Point end=waypoints[waypoints.size()-1];

    float curvature=get_curvature(waypoints[waypoints.size()-3], start, end);

    float dist=calc_dist(start,end,curvature,chassis);

    float deltaTime=dist/fmin(start.leftVelo,start.rightVelo);

    //^ Generates an extra end point to compensate for any path shortening
    waypoints.emplace_back(interpolate(start, end, 1+.01/deltaTime));

    //^ Write to file
    outLine<<waypoints[waypoints.size()-1].x<<" "<<waypoints[waypoints.size()-1].y<<" "<<waypoints[waypoints.size()-1].leftVelo<<" "<<waypoints[waypoints.size()-1].rightVelo<<std::endl;
    if(waypoints[waypoints.size()-1].instruction!=0) outLine<<waypoints[waypoints.size()-1].instruction<<std::endl;

    outLine.close();
    return;
}