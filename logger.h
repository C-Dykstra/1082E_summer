#pragma once
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
	Logger(std::string name, std::shared_ptr<float> param_x, std::shared_ptr<float> param_y);
	Logger(std::string name, std::shared_ptr<float> param_y);
};