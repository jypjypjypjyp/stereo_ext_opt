#ifndef lvio_fusion_FRONTEND_H
#define lvio_fusion_FRONTEND_H

#include "lvio_fusion/common.h"
#include "lvio_fusion/frame.h"

namespace lvio_fusion
{

typedef std::vector<cv::Point2f> Points;

class Frontend
{
public:
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend() {}

    bool AddFrame(Frame::Ptr frame);

    std::mutex mutex;
    Frame::Ptr current_frame;
    Frame::Ptr last_frame;
    Frame::Ptr last_keyframe;
    double init_time = 0;

private:
    void OptimizeEx(Points &ll, Points &rr);

    void EstimateD(Points &ll, Points &rr, std::vector<double> &dd);
};

} // namespace lvio_fusion

#endif // lvio_fusion_FRONTEND_H
