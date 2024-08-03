class MotionProfile{
    float voltage_speed_slope;
    float min_voltage;
    float max_accel;
    float max_decel;//< POSITIVE!!
    float max_speed;
    float mu;
    float weight;

    public:
    MotionProfile(float slope, float min, float a, float d, float s, float pweight, float pmu){}
    float get_accel(){
        return max_accel;
    }
    float get_decel(){
        return max_decel;
    }
    float get_min_voltage(){
        return min_voltage;
    }
    float get_max_speed(){
        return max_speed;
    }
    float get_v_s_slope(){
        return voltage_speed_slope;
    }
    float get_mu(){
        return mu;
    }
    float get_weight(){
        return weight;
    }
};
