#ifndef lvio_fusion_FRAME_H
#define lvio_fusion_FRAME_H

#include "lvio_fusion/common.h"
#include "lvio_fusion/visual/feature.h"

namespace lvio_fusion
{

class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;

    Frame();

    void AddFeature(visual::Feature::Ptr feature);

    void RemoveFeature(visual::Feature::Ptr feature);

    void Clear();

    static Frame::Ptr Create();

    static unsigned long current_frame_id;
    unsigned long id;
    double time;
    Frame::Ptr last_keyframe;
    cv::Mat image_left, image_right;
    visual::Features features_left;             // extracted features in left image
    visual::Features features_right;            // extracted features in right image
    SE3d pose;
};

typedef std::map<double, Frame::Ptr> Frames;

} // namespace lvio_fusion

#endif // lvio_fusion_FRAME_H
