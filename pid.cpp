#include <cmath>
#include <ctime>
#include <tuple>
#include <vector>
#include "motionProfile.h"

class PID{
    float kP;
    float kI;
    float kD;
    float integral;
    float prev_error;
    float max_error;

    short integralToggle=1;

    public:
    PID(float p, float i, float d, float max) : kP(p), kI(i), kD(d), max_error(max){}
    PID(std::tuple<float, float, float> consts, float max) : kP(get<0>(consts)), kI(get<1>(consts)), kD(get<2>(consts)), max_error(max){}

    float update(float error){
        integral+=prev_error+(error-prev_error)/2.0;
        if(error==0||prev_error==0||fabs(prev_error)/prev_error==fabs(error)/error) integral=0;
        float out=kP*error+kI*integral*integralToggle+kD*(error-prev_error);
        prev_error=error;
        return out;
    }
    float update(float error, float speed){
        integral+=prev_error+(error-prev_error)/2.0;
        if(error==0||prev_error==0||fabs(prev_error)/prev_error==fabs(error)/error) integral=0;
        float out=kP*error+kI*integral*integralToggle+kD*speed;
        prev_error=error;
        return out;
    }
    void reset(){
        integral=0;
        prev_error=0;
    }
    float get_max_error(){
        return max_error;
    }
    void toggle_kI(){
        (integralToggle==0)? integralToggle=1:integralToggle=0;
    }
    void toggle_kI(bool on){
        (on)? integralToggle=1:integralToggle=0;
    }
//~ Autotune

//> lateral
    float compute_lat_pid(std::vector<float> consts, MotionProfile profile, float ratio, float wheelRadius, float dt=.01){//return cost
        float rem_error=10;
        int iterations=0;
        PID internal(consts[0],consts[1],consts[2], max_error);
        int out=update(rem_error);
        float ITAE=0;
        while(rem_error>max_error&&out/profile.get_v_s_slope()>profile.get_min_voltage()){
            rem_error-=model_dist(out, ratio, wheelRadius, dt);
            float updated=update(rem_error);
            float test=(updated-out)/dt;
            if((test>=0&&test>profile.get_accel())){
                out+=profile.get_accel()*dt;
            }
            else if(test<profile.get_decel()){
                out-=profile.get_decel()*dt;
            }
            else{ out=updated; }
            //Compute Cost
            ITAE+=rem_error*iterations;
            iterations++;
        }
        return ITAE;
    }
    float compute_lat_pid(float p, float i, float d, MotionProfile profile, float ratio, float wheelRadius, float dt=.01){//return cost
        float rem_error=10;
        int iterations=0;
        PID internal(p,i,d, max_error);
        int out=update(rem_error);
        float ITAE=0;
        while(rem_error>max_error&&out/profile.get_v_s_slope()>profile.get_min_voltage()){
            rem_error-=model_dist(out, ratio, wheelRadius, dt);
            float updated=update(rem_error);
            float test=(updated-out)/dt;
            if((test>=0&&test>profile.get_accel())){
                out+=profile.get_accel()*dt;
            }
            else if(test<profile.get_decel()){
                out-=profile.get_decel()*dt;
            }
            else{ out=updated; }
            //Compute Cost
            ITAE+=rem_error*iterations;
            iterations++;
        }
        return ITAE;
    }
    float model_dist(float velo, int ratio, float wheelRadius, float dt){//velo should be profile rpm
        float speed = velo*ratio*wheelRadius;//profile/sec
        return speed*dt;//inches
    }

    void GRADIENT_DESCENT_LAT(float p, float i, float d, float alpha, float tolerance, float max_iterations, float epsilon, MotionProfile profile, float ratio, float wheelRadius){

        float kp=p;
        float ki=i;
        float kd=d;
        for(int i=0;i<max_iterations;i++){
            float cost=compute_lat_pid(p,i,d, profile, ratio, wheelRadius);
            float gradient_p=(compute_lat_pid(p+epsilon,i,d, profile, ratio, wheelRadius)-cost)/epsilon;
            float gradient_i=(compute_lat_pid(p,i+epsilon,d, profile, ratio, wheelRadius)-cost)/epsilon;
            float gradient_d=(compute_lat_pid(p,i,d+epsilon, profile, ratio, wheelRadius)-cost)/epsilon;
            kp-=alpha*gradient_p;
            ki-=alpha*gradient_i;
            kd-=alpha*gradient_d;

            if(fabs(gradient_p)<tolerance&&fabs(gradient_i)<tolerance&&fabs(gradient_d)<tolerance){
                break;
            }
        }
        kP=kp;
        kI=ki;
        kD=kd;
    }

    void TUNE_LAT(float p, float i, float d, float init_temp, std::pair<float,float> bounds, float cooling_rate, float max_iterations, 
        float alpha, float gradient_tolerance, float epsilon, MotionProfile profile, float ratio, float wheelRadius){

            std::vector<float>current_consts{p,i,d};
            float current_cost=compute_lat_pid(current_consts, profile, ratio, wheelRadius);
            float best_cost=current_cost;
            std::vector<float>best_consts=current_consts;

            std::vector<float> new_consts=current_consts;

            srand(time(0));

            for(int iteration=0;i<max_iterations;i++){

                double temp=std::max(init_temp * pow(cooling_rate, iteration), 1e-8);

                for (int i=0; i<new_consts.size(); i++) {
                    new_consts[i] += (( static_cast<double>(rand()) / RAND_MAX) * 2 - 1) * temp;
                    new_consts[i] = std::max(bounds.first, std::min(new_consts[i], bounds.second));
                }

                float new_cost=compute_lat_pid(new_consts, profile, ratio, wheelRadius);

                if (new_cost < current_cost || exp((current_cost - new_cost) / temp) > static_cast<double>(rand()) / RAND_MAX) {
                    current_consts = new_consts;
                    current_cost = new_cost;
                    

                    if (new_cost < best_cost) {
                        best_consts = new_consts;
                        best_cost = new_cost;
                    }
                }

            }

        GRADIENT_DESCENT_LAT(best_consts[0], best_consts[1], best_consts[2], alpha, gradient_tolerance, max_iterations, epsilon, profile, ratio, wheelRadius);

    }

//> angular
    float compute_ang_pid(float p, float i, float d, MotionProfile profile, float ratio, float wheelRadius, float width, float dt=.01){//return error
        float rem_error=2*M_PI;
        int iterations=0;
        PID internal(p,i,d, max_error);
        int out=update(rem_error);
        float ITAE=0;
        while(rem_error>max_error&&out/profile.get_v_s_slope()>profile.get_min_voltage()){
            rem_error-=model_ang_dist(out, ratio, wheelRadius, width, dt);
            float updated=update(rem_error);
            float test=(updated-out)/dt;
            if((test>=0&&test>profile.get_accel())){//TODO! Check signs
                out+=profile.get_accel()*dt;
            }
            else if(test<profile.get_decel()){
                out-=profile.get_decel()*dt;
            }
            else{ out=updated; }
            //Compute Cost
            ITAE+=rem_error*iterations;
            iterations++;
        }
        return ITAE;
    }
    float compute_ang_pid(std::vector<float> consts, MotionProfile profile, float ratio, float wheelRadius, float width, float dt=.01){//return error
        float rem_error=2*M_PI;
        int iterations=0;
        PID internal(consts[0], consts[1], consts[2], max_error);
        int out=update(rem_error);
        float ITAE=0;
        while(rem_error>max_error&&out/profile.get_v_s_slope()>profile.get_min_voltage()){
            rem_error-=model_ang_dist(out, ratio, wheelRadius, width, dt);
            float updated=update(rem_error);
            float test=(updated-out)/dt;
            if((test>=0&&test>profile.get_accel())){//TODO! Check signs
                out+=profile.get_accel()*dt;
            }
            else if(test<profile.get_decel()){
                out-=profile.get_decel()*dt;
            }
            else{ out=updated; }
            //Compute Cost
            ITAE+=rem_error*iterations;
            iterations++;
        }
        return ITAE;
    }
    float model_ang_dist(float velo, int ratio, float wheelRadius, float width, float dt){//speed should be profile rpm
        //> delta theta = (delta l - delta r)/width
        float dL=velo*ratio*wheelRadius*dt;
        // delta R = -delta L
        return 2*dL/width;
        //This means the sides should be run at ±dL/dt (PID output*ratio*wheelRadius)
    }
    
    void GRADIENT_DESCENT_ANG(float p, float i, float d, float alpha, float tolerance, float max_iterations, float epsilon, MotionProfile profile, float ratio, float wheelRadius, float width){

        float kp=p;
        float ki=i;
        float kd=d;
        for(int i=0;i<max_iterations;i++){
            float cost=compute_lat_pid(p,i,d, profile, ratio, wheelRadius);
            float gradient_p=(compute_ang_pid(p+epsilon,i,d, profile, ratio, wheelRadius, width)-cost)/epsilon;
            float gradient_i=(compute_ang_pid(p,i+epsilon,d, profile, ratio, wheelRadius, width)-cost)/epsilon;
            float gradient_d=(compute_ang_pid(p,i,d+epsilon, profile, ratio, wheelRadius, width)-cost)/epsilon;
            kp-=alpha*gradient_p;
            ki-=alpha*gradient_i;
            kd-=alpha*gradient_d;

            if(fabs(gradient_p)<tolerance&&fabs(gradient_i)<tolerance&&fabs(gradient_d)<tolerance){
                break;
            }
        }
        kP=kp;
        kI=ki;
        kD=kd;
    }

    void TUNE_ANG(float p, float i, float d, float init_temp, std::pair<float,float> bounds, float cooling_rate, float max_iterations, 
        float alpha, float gradient_tolerance, float epsilon, MotionProfile profile, float ratio, float wheelRadius, float width){

            std::vector<float>current_consts{p,i,d};
            float current_cost=compute_ang_pid(current_consts, profile, ratio, wheelRadius, width);
            float best_cost=current_cost;
            std::vector<float>best_consts=current_consts;

            std::vector<float> new_consts=current_consts;

            srand(time(0));

            for(int iteration=0;i<max_iterations;i++){

                double temp=std::max(init_temp * pow(cooling_rate, iteration), 1e-8);

                for (int i=0; i<new_consts.size(); i++) {
                    new_consts[i] += (( static_cast<double>(rand()) / RAND_MAX) * 2 - 1) * temp;
                    new_consts[i] = std::max(bounds.first, std::min(new_consts[i], bounds.second));
                }

                float new_cost=compute_ang_pid(new_consts, profile, ratio, wheelRadius, width);

                if (new_cost < current_cost || exp((current_cost - new_cost) / temp) > static_cast<double>(rand()) / RAND_MAX) {
                    current_consts = new_consts;
                    current_cost = new_cost;
                    

                    if (new_cost < best_cost) {
                        best_consts = new_consts;
                        best_cost = new_cost;
                    }
                }

            }

        GRADIENT_DESCENT_ANG(best_consts[0], best_consts[1], best_consts[2], alpha, gradient_tolerance, max_iterations, epsilon, profile, ratio, wheelRadius, width);

    }
//~
};