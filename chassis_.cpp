#include <cmath>
#include <math.h>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include "motionProfile.h"
#include "pid.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <vector>
#include <iostream>
#include <fstream>

class Chassis{//TODO! MAKE Direction REVERSIBLE
    pros::MotorGroup& l;
    pros::MotorGroup& r;
    float width;
    float ratio;
    
    PID angPID;
    PID latPID;

    pros::Rotation hEnc;
    pros::Rotation vEnc;
    pros::IMU inertial;
    float wheelRadius;
    float vertOffset;//left/right
    float horizOffset;//forwards/backwards

    double x;
    double y;
    double theta;

//~ pseudo-kalman filter data
    float dt=.01;//seconds

    float prevGainTheta=0.2;
    float prevGainLat=0;
    float prevGainCross=0;

    float prevValTheta=0;
    float prevValLat=0;
    float prevValCross=0;
    
    float prevVeloTheta=0;
    float prevVeloLat=0;
    float prevVeloCross=0;

    float prevAccelTheta=0;
    float prevAccelLat=0;
    float prevAccelCross=0;
//~

    MotionProfile chassisProfile;
    MotionProfile realMax;

    void odom(float xp=0,float yp=0,float t=0){
        x=xp;
        y=yp;
        theta=t;

        hEnc.reset();
        vEnc.reset();
        inertial.reset();

        while(true){
            float deltaTheta=inertial.get_heading();
            float deltaThetaBelief=prevVeloTheta*dt+0.5*prevAccelTheta*dt*dt;
            float finalDeltaTheta=prevGainTheta*deltaThetaBelief+deltaTheta*(1-prevGainTheta);

            float calcAngle=theta+finalDeltaTheta/2.0;

            float vDist=vEnc.get_angle()/360.0*2.0*M_PI*wheelRadius;
            float hDist=hEnc.get_angle()/360.0*2.0*M_PI*wheelRadius;

            float vDistBelief=prevVeloLat*dt+.5*prevAccelLat*dt*dt;
            float hDistBelief=prevVeloCross*dt+.5*prevAccelCross*dt*dt;
            
            float finalLatDist=prevGainLat*vDistBelief+(1-prevGainLat)*vDist;
            float finalCrossDist=prevGainCross*hDistBelief+(1-prevGainLat)*hDist;

            float radius=vDist/deltaTheta-vertOffset;
            x+=radius*finalDeltaTheta*cos(calcAngle)+hDist*sin(calcAngle);
            y+=radius*finalDeltaTheta*sin(calcAngle)+hDist*cos(calcAngle);
            theta=fmod(theta+finalDeltaTheta, 2*M_PI);

        //~ Update prediction values
            float currentValTheta=finalDeltaTheta;
            float currentValLat=finalLatDist;
            float currentValCross=finalCrossDist;

            float currentVeloTheta=(currentValTheta-prevValTheta)/dt;
            float currentVeloLat=(currentValLat-prevValLat)/dt;
            if(currentVeloLat>realMax.get_max_speed()){currentVeloLat=realMax.get_max_speed();}
            float currentVeloCross=(currentValCross-prevValCross)/dt;

            prevAccelTheta=(currentVeloTheta-prevVeloTheta)/dt;
            prevAccelLat=(currentVeloLat-prevVeloLat)/dt;
            prevAccelCross=(currentVeloCross-prevVeloCross)/dt;

            prevVeloTheta=currentVeloTheta;
            prevVeloLat=currentVeloLat;
            prevVeloCross=currentVeloCross;

            prevValTheta=currentValTheta;
            prevValLat=currentValLat;
            prevValCross=currentValCross;
        //~

            vEnc.reset();
            hEnc.reset();
            inertial.reset();
            pros::delay(10);
        }
    }

    float calc_dist_to_point(std::pair<float, float> point){
        return sqrt( (x-point.first)*(x-point.first)+(y-point.second)*(y-point.second) );
    }
    float calc_dist_to_point(std::tuple<float, float, float> point){
        return sqrt( (x-get<0>(point))*(x-get<0>(point))+(y-get<1>(point))*(y-get<1>(point)) );
    }
    float calc_dist(std::pair<float, float> point, std::pair<float,float> point2){
        return sqrt( (point2.first-point.first)*(point2.first-point.first)+(point2.second-point.second)*(point2.second-point.second) );
    }

    public:
    Chassis(pros::MotorGroup& left, pros::MotorGroup& right, float Width, float Ratio, PID ang, PID lat, pros::Rotation h, pros::Rotation v, pros::IMU i, float vert, float horiz, MotionProfile profile, MotionProfile profile2, float r=2.75/2.0) : 
        l(left), r(right), width(Width), ratio(Ratio), angPID(ang), latPID(lat), hEnc(h), vEnc(v), inertial(i), vertOffset(vert), horizOffset(horiz), chassisProfile(profile), realMax(profile2), wheelRadius(r) {
            pros::Task odomTask([this]{ odom(); });
    }

    std::tuple<double, double, double> get_coords(){
        return std::make_tuple(x,y,theta);
    }

    void move_pid_lat(float dist){
        std::pair<float, float> point=std::make_pair(x+dist*cos(theta), y+dist*sin(theta));
        float deltaTheta=0;
        while(calc_dist_to_point(point)*cos(atan2(point.first-x, point.second-y)-theta)>latPID.get_max_error()){
            this->move_velocity(latPID.update(dist));
            pros::delay(10);
        }
        latPID.reset();
    }

    void move_pid_ang(float deg){
        float target=theta+deg;
        while(theta-deg>angPID.get_max_error()){
            l.move_velocity(angPID.update(deg));
            r.move_velocity(-angPID.update(deg));
            pros::delay(10);
        }
        angPID.reset();
    }

    void pid_swing(float deg, float middleRadius){
        float target=theta+deg;
        float rightLen;
        float leftLen;
        float innerRadius=middleRadius-width/2.0;

        if(deg>=0){
            rightLen=deg*(middleRadius-width/2.0);
            leftLen=deg*(middleRadius+width/2.0);
            while(rightLen>latPID.get_max_error()||leftLen>latPID.get_max_error()){
                float out=latPID.update(leftLen);
                l.move_velocity(out);
                r.move_velocity(out*innerRadius/(width+innerRadius));
                leftLen-=out*ratio*wheelRadius*.01;
                rightLen-=out*innerRadius/(width+innerRadius)*ratio*wheelRadius*.01;
                pros::delay(10);
            }
        }
        else{
            rightLen=deg*(middleRadius+width/2.0);
            leftLen=deg*(middleRadius-width/2.0);
            while(rightLen>latPID.get_max_error()||leftLen>latPID.get_max_error()){
                float out=latPID.update(leftLen);
                l.move_velocity(out*innerRadius/(width+innerRadius));
                r.move_velocity(out);
                rightLen-=out*ratio*wheelRadius*.01;
                leftLen-=out*innerRadius/(width+innerRadius)*ratio*wheelRadius*.01;
                pros::delay(10);
            }
        }

        latPID.reset();
    }

//~ Boomerang
public:
    void boomerang(std::tuple<float,float,float> point, float gLead, float dLead, float minspeed=0, float maxspeed=127, float pswitch=7.5){
        std::pair<float, float> carrot=calcCarrot(point,gLead, dLead);

        if(dLead!=1) boomerang(std::make_tuple(carrot.first,carrot.second,0), gLead, dLead, minspeed, pswitch);

        float distToCarrot=calc_dist_to_point(carrot);
        float distToEnd=calc_dist_to_point(point);

        std::pair<float, float> out;

        if(dLead==1){
			while(distToCarrot>pswitch){
				float angularPower=angPID.update(get<2>(point)-theta);
				float linearPower=latPID.update(calc_dist_to_point(point), (vEnc.get_velocity()));

				float initOut=fabs(linearPower)+fabs(angularPower);

                float speed=chassisProfile.get_max_speed();

				if(initOut>speed){
                    linearPower-=initOut-speed;
                }

                out.first=linearPower+angularPower;
                out.second=linearPower-angularPower;

                if(fmin(fabs(out.first),fabs(out.second))==0){
					out.first+=.0001;
					out.second+=.0001;
				}

				if(fmin(fabs(out.first/out.second),fabs(out.second/out.first))<1.0/4.0){//TODO CALCULATE MAX SPEED FOR CURVATURE!!!
                //<TO TUNE
					out.first/=3.0;
					out.second/=3.0;
				}

                float ratio=out.second/out.first;
			    if(out.first<minspeed) out.first=minspeed;
			    if(out.second<minspeed) out.second=minspeed*ratio;

                l.move_velocity(out.first);
                r.move_velocity(out.second);

                distToCarrot=calc_dist_to_point(carrot);

                pros::delay(10);
			}
        }
        else{
            while(true){
                std::pair<float,float> carrot=calcCarrot(point,gLead, 1);
				float angularPower=angPID.update(get<2>(point)-theta);
				float linearPower=latPID.update(calc_dist_to_point(point), (vEnc.get_velocity()));

				float initOut=fabs(linearPower)+fabs(angularPower);

                float speed=chassisProfile.get_max_speed();

				if(initOut>speed){
                    linearPower-=initOut-speed;
                }

                out.first=linearPower+angularPower;
                out.second=linearPower-angularPower;

                if(fmin(fabs(out.first),fabs(out.second))==0){
					out.first+=.0001;
					out.second+=.0001;
				}

				if(fmin(fabs(out.first/out.second),fabs(out.second/out.first))<1.0/4.0){//TODO CALCULATE MAX SPEED FOR CURVATURE!!!
                //<TO TUNE
					out.first/=3.0;
					out.second/=3.0;
				}

                float ratio=out.second/out.first;
			    if(out.first<minspeed) out.first=minspeed;
			    if(out.second<minspeed) out.second=minspeed*ratio;

                float trigNum=get<2>(point)-(5.0*M_PI/2.0);

                float min_volt=chassisProfile.get_min_voltage();

                if( (fabs(out.first)<min_volt&&fabs(out.second)<min_volt)  || 
                    ((x-get<0>(point))*sin(trigNum)+(y-get<1>(point))*cos(trigNum)>(x-get<0>(point))*cos(trigNum)-(y-get<1>(point))*sin(trigNum)&&calc_dist_to_point(point)<2) ){//TODO Tune
					break;
				}//Semicircle and min voltage exit conditions

                l.move_velocity(out.first);
                r.move_velocity(out.second);

                distToCarrot=calc_dist_to_point(carrot);

                pros::delay(10);
			}
            latPID.reset();
            angPID.reset();
        }
    }

    private:
    std::pair<float,float> calcCarrot(std::tuple<float,float,float> point, float gLead, float dLead=0){
		std::pair<float,float> out;
		float dist=gLead*calc_dist_to_point(std::make_pair(get<0>(point),get<1>(point)))*(1-dLead);
		out.first=x-dist*cos(theta);
		out.second=y-dist*sin(theta);
		return out;
	}
//~

    public:
//~ Pure Pursuit
    void follow_path(float ld, std::vector<std::tuple<float,float,float>> path){
        int lastFoundIndex=0;
        while(true){
            for(int i=path.size()-1;i>lastFoundIndex;i--){//Search path backwards
                float x0Off=get<0>(path.at(i))-x;
                float x1Off=get<0>(path.at(i+1))-x;
                float y0Off=get<1>(path.at(i))-y;
                float y1Off=get<1>(path.at(i+1))-y;

                float dx=x1Off-x0Off;
                float dy=y1Off-y0Off;

                float r=sqrt(dx*dx+dy*dy);
                float D=x0Off*y1Off-x1Off*y0Off;
                float discriminant=ld*ld*r*r-D*D;

                if(discriminant<0) continue;

                //chase point
                float pointX0=(D*dy+ (fabs(dy)/dy) *dx*sqrt(discriminant))/(r*r)+x;
                float pointX1=(D*dy- (fabs(dy)/dy) *dx*sqrt(discriminant))/(r*r)+y;
                float pointY0=(-D*dx+fabs(dy)*sqrt(discriminant))/(r*r);
                float pointY1=(-D*dx-fabs(dy)*sqrt(discriminant))/(r*r);

                bool p1Valid=false;
                bool p2Valid=false;
                if(pointX0==fmin( fmax( pointX0,fmin(x0Off,x1Off) ), fmax(x0Off,x1Off) )&&pointY0==fmin( fmax( pointY0,fmin(y0Off,y1Off) ), fmax(y0Off,y1Off) )){
                    p1Valid=true;
                }
                if(pointX1==fmin( fmax( pointX1,fmin(x0Off,x1Off) ), fmax(x0Off,x1Off) )&&pointY1==fmin( fmax( pointY1,fmin(y0Off,y1Off) ), fmax(y0Off,y1Off) )){
                    p2Valid=true;
                }

                if(!p1Valid&&!p2Valid) continue;

                lastFoundIndex=i;
                break;
            }

            //Follow lastFoundIndex point
            if(lastFoundIndex==path.size()-1) break;

            float xVal=get<0>(path.at(lastFoundIndex));
            float yVal= get<1>(path.at(lastFoundIndex));

            float angError=atan2(xVal-x,yVal-y)+M_PI/2.0;
            if(angError<0) angError+=2*M_PI;
            angError-=theta;
            if(angError>M_PI) angError=2.0*M_PI-angError;

            float angVelo=angPID.update(angError);
            float linVelo=chassisProfile.get_max_speed();
            float initOutRight=linVelo-angVelo;
            float initOutLeft=linVelo+angVelo;

            if(initOutRight>get<2>(path.at(lastFoundIndex))){
                float over=initOutRight-get<2>(path.at(lastFoundIndex));
                float scale=(initOutRight-over)/initOutRight;
                initOutRight*=scale;
                initOutLeft*=scale;
            }
            if(initOutLeft>get<2>(path.at(lastFoundIndex))){
                float over=initOutLeft-get<2>(path.at(lastFoundIndex));
                float scale=(initOutLeft-over)/initOutLeft;
                initOutRight*=scale;
                initOutLeft*=scale;
            }

            r.move_velocity(initOutRight);
            l.move_velocity(initOutLeft);
            pros::delay(10);
        }

        boomerang(path.at(path.size()-1), 1, 1);//TODO Tune g- and d-leads

        angPID.reset();
    }

    std::vector<std::tuple<float,float,float>> lerp(int points, int index, std::vector<std::tuple<float,float,float>> path){
        std::vector<std::tuple<float,float,float>> out;
        for(int i=0;i<points;i++){
            float t=1.0/(points+1.0);
            
            float outX;
            float outY;
            float outV;

            if(index>=1&&index<path.size()-1){
                outX=(1-t)*get<0>(path.at(index-1)) + (1-t)*get<0>(path.at(index))*t + (1-t)*get<0>(path.at(index+1))*t*t + (1-t)*get<0>(path.at(index+1))*t*t*t;
                outY=(1-t)*get<1>(path.at(index-1)) + (1-t)*get<1>(path.at(index))*t + (1-t)*get<1>(path.at(index+1))*t*t + (1-t)*get<1>(path.at(index+1))*t*t*t;
                outV=(1-t)*get<2>(path.at(index-1)) + (1-t)*get<2>(path.at(index))*t + (1-t)*get<2>(path.at(index+1))*t*t + (1-t)*get<2>(path.at(index+1))*t*t*t;
            }
            else if(index==0){
                outX=(1-t)*get<0>(path.at(index)) + (1-t)*get<0>(path.at(index+1))*t + (1-t)*get<0>(path.at(index+2))*t*t + (1-t)*get<0>(path.at(index+3))*t*t*t;
                outY=(1-t)*get<1>(path.at(index)) + (1-t)*get<1>(path.at(index+1))*t + (1-t)*get<1>(path.at(index+2))*t*t + (1-t)*get<1>(path.at(index+3))*t*t*t;
                outV=(1-t)*get<2>(path.at(index)) + (1-t)*get<2>(path.at(index+1))*t + (1-t)*get<2>(path.at(index+2))*t*t + (1-t)*get<2>(path.at(index+3))*t*t*t;
            }
            else if(index==path.size()-1){
                outX=(1-t)*get<0>(path.at(index-3)) + (1-t)*get<0>(path.at(index-2))*t + (1-t)*get<0>(path.at(index-1))*t*t + (1-t)*get<0>(path.at(index))*t*t*t;
                outY=(1-t)*get<1>(path.at(index-3)) + (1-t)*get<1>(path.at(index-2))*t + (1-t)*get<1>(path.at(index-1))*t*t + (1-t)*get<1>(path.at(index))*t*t*t;
                outV=(1-t)*get<2>(path.at(index-3)) + (1-t)*get<2>(path.at(index-2))*t + (1-t)*get<2>(path.at(index-1))*t*t + (1-t)*get<2>(path.at(index))*t*t*t;
            }
            out.push_back(std::make_tuple(outX,outY,outV));
        }
        return out;
    }

    /*
    * @param fileName: file name, not including ".txt"
    */
    std::vector<std::tuple<float,float,float>> parse_path(std::string fileName){
        //& Path format given is x, y, speed (rpm), heading (deg) OR if short, x, y, speed
        std::string filePath = "\\usd\\"+fileName+".txt";
        std::ifstream file(filePath);
        std::vector<std::tuple<float,float,float>> out;
        
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << fileName << std::endl;
            return out;
        }
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std::string line;
        while(std::getline(file,line)){
            std::stringstream ss(line);
            std::string info[4];
            int index=0;
            while(std::getline(ss, info[index], ' ')){index++;}
            out.push_back(std::make_tuple(std::stof(info[0]),std::stof(info[1]), std::stof(info[2])));
        }
        return out;
    }

//~

    float get_width(){
        return width;
    }

//~ Motor group methods
    void move(float voltage){
        l.move(voltage);
        r.move(voltage);
    }
    void move_absolute(float position, float velocity){
        l.move_absolute(position, velocity);
        r.move_absolute(position, velocity);
    }
    void move_relative(float position, float velocity){
        l.move_relative(position, velocity);
        r.move_relative(position, velocity);
    }
    void move_velocity(float velocity){
        l.move_velocity(velocity);
        r.move_velocity(velocity);
    }
    void move_voltage(float voltage){
        l.move_voltage(voltage);
        r.move_voltage(voltage);
    }
    void brake(){
        l.brake();
        r.brake();
    }
    std::pair<float, float> get_target_velocities(){
        std::pair<float, float> out;
        out.first=l.get_target_velocity();
        out.second=r.get_target_velocity();
        return out;
    }
    float get_target_position(bool side, int index){
        if(!side){
            return l.get_target_position(index);
        }
        return r.get_target_position(index);
    }
    std::pair<std::vector<float>, std::vector<float>> get_target_position_all(){
        std::pair<std::vector<float>, std::vector<float>> out;
            std::vector<double> left=l.get_target_position_all();
            std::vector<double> right=r.get_target_position_all();
            std::vector<float> outLeft(left.begin(), left.end());
            std::vector<float> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;       
        return out;
    }
    float get_actual_velocity(bool side, int index){
        if(!side){
            return l.get_target_position(index);
        }
        return r.get_target_position(index);
    }
    std::pair<std::vector<float>, std::vector<float>> get_actual_velocity_all(){
        std::pair<std::vector<float>, std::vector<float>> out;
            std::vector<double> left=l.get_actual_velocity_all();
            std::vector<double> right=r.get_actual_velocity_all();
            std::vector<float> outLeft(left.begin(), left.end());
            std::vector<float> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;       
        return out;
    }
    float get_current_draw(bool side, int index){
        if(!side){
            return l.get_current_draw(index);
        }
        return r.get_current_draw(index);
    }
    std::pair<std::vector<float>, std::vector<float>> get_current_draw_all(){
        std::pair<std::vector<float>, std::vector<float>> out;
            std::vector<int> left=l.get_current_draw_all();
            std::vector<int> right=r.get_current_draw_all();
            std::vector<float> outLeft(left.begin(), left.end());
            std::vector<float> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;
        return out;
    }
    int get_direction(bool side, int index){
        if(!side){
            return l.get_direction(index);
        }
        return r.get_direction(index);
    }
    std::pair<std::vector<int>, std::vector<int>> get_direction_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
        out.first=l.get_direction_all();
        out.second=r.get_direction_all();        
        return out;
    }
    float get_efficiency(bool side, int index){
        if(!side){
            return l.get_efficiency(index);
        }
        return r.get_efficiency(index);
    }
    std::pair<std::vector<float>, std::vector<float>> get_efficiency_all(){
        std::pair<std::vector<float>, std::vector<float>> out;
            std::vector<double> left=l.get_efficiency_all();
            std::vector<double> right=r.get_efficiency_all();
            std::vector<float> outLeft(left.begin(), left.end());
            std::vector<float> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;       
        return out;        
    }
    int get_faults(bool side, int index){
        if(!side){
            return l.get_faults(index);
        }
        return r.get_faults(index);
    }
    std::pair<std::vector<int>, std::vector<int>> get_faults_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
            std::vector<std::uint32_t> left=l.get_faults_all();
            std::vector<std::uint32_t> right=r.get_faults_all();
            std::vector<int> outLeft(left.begin(), left.end());
            std::vector<int> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;
        return out;
    }
    int get_flags(bool side, int index){
        if(!side){
            return l.get_flags(index);
        }
        return r.get_flags(index);
    }
    std::pair<std::vector<int>, std::vector<int>> get_flags_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
            std::vector<std::uint32_t> left=l.get_flags_all();
            std::vector<std::uint32_t> right=r.get_flags_all();
            std::vector<int> outLeft(left.begin(), left.end());
            std::vector<int> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;
        return out;
    }
    float get_position(bool side, int index){
        if(!side){
            return l.get_position(index);
        }
        return r.get_position(index);
    }
    std::pair<std::vector<float>, std::vector<float>> get_position_all(){
        std::pair<std::vector<float>, std::vector<float>> out;
            std::vector<double> left=l.get_position_all();
            std::vector<double> right=r.get_position_all();
            std::vector<float> outLeft(left.begin(), left.end());
            std::vector<float> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;       
        return out;
    }
    float get_power(bool side, int index){
        if(!side){
            return l.get_power(index);
        }
        return r.get_power(index);
    }
    std::pair<std::vector<float>, std::vector<float>> get_power_all(){
        std::pair<std::vector<float>, std::vector<float>> out;
            std::vector<double> left=l.get_power_all();
            std::vector<double> right=r.get_power_all();
            std::vector<float> outLeft(left.begin(), left.end());
            std::vector<float> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;       
        return out;
    }
    int get_raw_position(bool side, int index, uint32_t* timestamp){
        if(!side){
            return l.get_raw_position(timestamp, index);
        }
        return r.get_raw_position(timestamp, index);
    }
    std::pair<std::vector<int>, std::vector<int>> get_raw_position_all(uint32_t* timestamp){
        std::pair<std::vector<int>, std::vector<int>> out;
        out.first=l.get_raw_position_all(timestamp);
        out.second=r.get_raw_position_all(timestamp);        
        return out;
    }
    float get_temperature(bool side, int index){
        if(!side){
            return l.get_temperature(index);
        }
        return r.get_temperature(index);
    }
    std::pair<std::vector<float>, std::vector<float>> get_temperature_all(){
        std::pair<std::vector<float>, std::vector<float>> out;
            std::vector<double> left=l.get_temperature_all();
            std::vector<double> right=r.get_temperature_all();
            std::vector<float> outLeft(left.begin(), left.end());
            std::vector<float> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;       
        return out;
    }
    float get_torque(bool side, int index){
        if(!side){
            return l.get_torque(index);
        }
        return r.get_torque(index);
    }
    std::pair<std::vector<float>, std::vector<float>> get_torque_all(){
        std::pair<std::vector<float>, std::vector<float>> out;
            std::vector<double> left=l.get_torque_all();
            std::vector<double> right=r.get_torque_all();
            std::vector<float> outLeft(left.begin(), left.end());
            std::vector<float> outRight(right.begin(), right.end());     
        out.first=outLeft;
        out.second=outRight;       
        return out;
    }
    float get_voltage(bool side, int index){
        if(!side){
            return l.get_voltage(index);
        }
        return r.get_voltage(index);
    }
    std::pair<std::vector<int>, std::vector<int>> get_voltage_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
        out.first=l.get_voltage_all();
        out.second=r.get_voltage_all();        
        return out;
    }
    int is_over_current(bool side, int index){
        if(!side){
            return l.is_over_current(index);
        }
        return r.is_over_current(index);
    }
    std::pair<std::vector<int>, std::vector<int>> is_over_current_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
        out.first=l.is_over_current_all();
        out.second=r.is_over_current_all();        
        return out;
    }
    int is_over_temp(bool side, int index){
        if(!side){
            return l.is_over_temp(index);
        }
        return r.is_over_temp(index);
    }
    std::pair<std::vector<int>, std::vector<int>> is_over_temp_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
        out.first=l.is_over_temp_all();
        out.second=r.is_over_temp_all();        
        return out;
    }
    pros::MotorBrake get_brake_mode(bool side, int index){
        if(!side){
            return l.get_brake_mode(index);
        }
        return r.get_brake_mode(index);
    }
    std::pair<std::vector<pros::MotorBrake>, std::vector<pros::MotorBrake>> get_brake_mode_all(){
        std::pair<std::vector<pros::MotorBrake>, std::vector<pros::MotorBrake>> out;
        out.first=l.get_brake_mode_all();
        out.second=r.get_brake_mode_all();        
        return out;
    }
    int get_current_limit(bool side, int index){
        if(!side){
            return l.get_current_limit(index);
        }
        return r.get_current_limit(index);
    }
    std::pair<std::vector<int>, std::vector<int>> get_current_limit_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
        out.first=l.get_current_limit_all();
        out.second=r.get_current_limit_all();        
        return out;
    }
    pros::MotorUnits get_encoder_units(bool side, int index){
        if(!side){
            return l.get_encoder_units(index);
        }
        return r.get_encoder_units(index);
    }
    std::pair<std::vector<pros::MotorUnits>, std::vector<pros::MotorUnits>> get_encoder_units_all(){
        std::pair<std::vector<pros::MotorUnits>, std::vector<pros::MotorUnits>> out;
        out.first=l.get_encoder_units_all();
        out.second=r.get_encoder_units_all();        
        return out;
    }
    pros::MotorGears get_gearing(bool side, int index){
        if(!side){
            return l.get_gearing(index);
        }
        return r.get_gearing(index);
    }
    std::pair<std::vector<pros::MotorGears>, std::vector<pros::MotorGears>> get_gearing_all(){
        std::pair<std::vector<pros::MotorGears>, std::vector<pros::MotorGears>> out;
        out.first=l.get_gearing_all();
        out.second=r.get_gearing_all();        
        return out;
    }
    int get_port(bool side, int index){
        if(!side){
            return l.get_port(index);
        }
        return r.get_port(index);
    }
    std::pair<std::vector<int8_t>, std::vector<int8_t>> get_port_all(){
        std::pair<std::vector<int8_t>, std::vector<int8_t>> out;
        out.first=l.get_port_all();
        out.second=r.get_port_all();        
        return out;
    }
    int get_voltage_limit(bool side, int index){
        if(!side){
            return l.get_voltage_limit(index);
        }
        return r.get_voltage_limit(index);
    }
    std::pair<std::vector<int>, std::vector<int>> get_voltage_limit_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
        out.first=l.get_voltage_limit_all();
        out.second=r.get_voltage_limit_all();        
        return out;
    }
    int is_reversed(bool side, int index){
        if(!side){
            return l.is_reversed(index);
        }
        return r.is_reversed(index);
    }
    std::pair<std::vector<int>, std::vector<int>> is_reversed_all(){
        std::pair<std::vector<int>, std::vector<int>> out;
        out.first=l.is_reversed_all();
        out.second=r.is_reversed_all();        
        return out;
    }
    int set_brake_mode(bool side, int index, pros::MotorBrake mode){
        if(!side){
            return l.set_brake_mode(mode, index);
        }
        return r.set_brake_mode(mode, index);
    }
    std::pair<int, int> set_brake_mode_all(pros::MotorBrake mode){
        std::pair<int, int> out;
        out.first=l.set_brake_mode_all(mode);
        out.second=r.set_brake_mode_all(mode);        
        return out;
    }
    int set_current_limit(bool side, int index, int limit){
        if(!side){
            return l.set_current_limit(limit, index);
        }
        return r.set_current_limit(limit, index);
    }
    std::pair<int, int> set_current_limit_all(int limit){
        std::pair<int, int> out;
        out.first=l.set_current_limit_all(limit);
        out.second=r.set_current_limit_all(limit);        
        return out;
    }
    int set_encoder_units(bool side, int index, pros::MotorUnits units){
        if(!side){
            return l.set_encoder_units(units, index);
        }
        return r.set_encoder_units(units, index);
    }
    std::pair<int, int> set_encoder_units_all(pros::MotorUnits units){
        std::pair<int, int> out;
        out.first=l.set_encoder_units_all(units);
        out.second=r.set_encoder_units_all(units);        
        return out;
    }
    int set_gearing(bool side, int index, pros::MotorGears gearset){
        if(!side){
            return l.set_gearing(gearset, index);
        }
        return r.set_gearing(gearset, index);
    }
    std::pair<int, int> set_gearing_all(pros::MotorGears gearset){
        std::pair<int, int> out;
        out.first=l.set_gearing_all(gearset);
        out.second=r.set_gearing_all(gearset);        
        return out;
    }
    int set_reversed(bool side, int index, bool reversed){
        if(!side){
            return l.set_reversed(reversed, index);
        }
        return r.set_reversed(reversed, index);
    }
    std::pair<int, int> set_reversed_all(bool reversed){
        std::pair<int, int> out;
        out.first=l.set_reversed_all(reversed);
        out.second=r.set_reversed_all(reversed);        
        return out;
    }
    int set_voltage_limit(bool side, int index, int limit){
        if(!side){
            return l.set_voltage_limit(limit, index);
        }
        return r.set_current_limit(limit, index);
    }
    std::pair<int, int> set_voltage_limit_all(int limit){
        std::pair<int, int> out;
        out.first=l.set_voltage_limit_all(limit);
        out.second=r.set_voltage_limit_all(limit);        
        return out;
    }
    int set_zero_position(bool side, int index, float pos){
        if(!side){
            return l.set_zero_position(pos, index);
        }
        return r.set_zero_position(pos, index);
    }
    std::pair<int, int> set_zero_position_all(float pos){
        std::pair<int, int> out;
        out.first=l.set_zero_position_all(pos);
        out.second=r.set_zero_position_all(pos);        
        return out;
    }
    int tare_position(bool side, int index){
        if(!side){
            return l.tare_position(index);
        }
        return r.tare_position(index);
    }
    std::pair<int, int> tare_position_all(int limit){
        std::pair<int, int> out;
        out.first=l.tare_position_all();
        out.second=r.tare_position_all();        
        return out;
    }
    int size(bool side){
        if(!side){
            return l.size();
        }
        return r.size();
    }
    std::pair<int, int> size(){
        std::pair<int, int> out;
        out.first=l.size();
        out.second=r.size();      
        return out;
    }
    void append(bool side, pros::AbstractMotor& motor){
        if(!side){
            l+=motor;
        }
        r+=motor;
    }
    void erase_port(bool side, int port){
        if(!side){
            l.erase_port(port);
        }
        r.erase_port(port);
    }
//~
};