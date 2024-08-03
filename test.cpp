#include "pros/rtos.hpp"
#include <mutex>
#include <condition_variable>

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