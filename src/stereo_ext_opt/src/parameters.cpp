#include "parameters.h"

string IMAGE0_TOPIC, IMAGE1_TOPIC;

void read_parameters(string config_file)
{
    FILE *f = fopen(config_file.c_str(), "r");
    if (f == NULL)
    {
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(f);

    cv::FileStorage settings(config_file, cv::FileStorage::READ);
    if (!settings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
    }

    settings["image0_topic"] >> IMAGE0_TOPIC;
    settings["image1_topic"] >> IMAGE1_TOPIC;
    settings.release();
}
