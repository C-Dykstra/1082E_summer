#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>

class Logger{
	std::string path;
	std::ofstream file;

	std::shared_ptr<float> x;
	std::shared_ptr<float> y;

	public:
	Logger(std::string name, std::shared_ptr<float> param_x, std::shared_ptr<float> param_y){
		path="/usd/"+name+".txt";
		file=std::ofstream(path);
		x=param_x;
		y=param_y;
		pros::Task logger([this]{
			while(true){
				file<<" "<<*x<<" "<<*y<<std::endl;
				pros::delay(10);
			}
		});
	}
	Logger(std::string name, std::shared_ptr<float> param_y){
		path="/usd/"+name+".txt";
		file=std::ofstream(path);
		x=nullptr;
		y=param_y;
		pros::Task logger([this]{
			int iterations=0;
			while(true){
				file<<" "<<iterations*.01<<" "<<*y<<std::endl;
				pros::delay(10);
			}
		});
	}
};