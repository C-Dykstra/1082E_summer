#pragma once

class PID{
    float k_p;
    float k_i;
    float k_d;
    float integral;
    float prev_error;
    float max_error;
    public:
    PID(float p, float i, float d){}

    float update(float error);
    float update(float error, float speed);
    void reset(){}
    float get_max_error();
    void toggle_kI();
    void toggle_kI(bool on);
};