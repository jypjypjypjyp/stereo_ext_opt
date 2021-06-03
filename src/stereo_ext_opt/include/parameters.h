#ifndef lvio_fusion_PARAMETERS_H
#define lvio_fusion_PARAMETERS_H

#include <ros/ros.h>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

extern string IMAGE0_TOPIC, IMAGE1_TOPIC;

void read_parameters(std::string config_file);

#endif // lvio_fusion_PARAMETERS_H