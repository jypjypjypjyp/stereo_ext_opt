
#ifndef lvio_fusion_VISUAL_ODOMETRY_H
#define lvio_fusion_VISUAL_ODOMETRY_H

#include "lvio_fusion/common.h"
#include "lvio_fusion/frontend.h"

namespace lvio_fusion
{

class Estimator
{
public:
    typedef std::shared_ptr<Estimator> Ptr;

    Estimator(std::string &config_path);

    void InputImage(double time, cv::Mat &left_image, cv::Mat &right_image);

    bool Init();

    Frontend::Ptr frontend;

private:
    std::string config_file_path_;
};
} // namespace lvio_fusion

#endif // lvio_fusion_VISUAL_ODOMETRY_H
