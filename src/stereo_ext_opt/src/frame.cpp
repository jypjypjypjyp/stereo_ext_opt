#include "lvio_fusion/frame.h"
#include "lvio_fusion/visual/camera.h"
#include "lvio_fusion/visual/landmark.h"

namespace lvio_fusion
{

unsigned long Frame::current_frame_id = 0;

Frame::Frame()
{
}

Frame::Ptr Frame::Create()
{
    Frame::Ptr new_frame(new Frame);
    new_frame->id = current_frame_id + 1;
    return new_frame;
}

void Frame::AddFeature(visual::Feature::Ptr feature)
{
    auto landmark = feature->landmark.lock();
    assert(feature->frame.lock()->id == id && landmark);
    if (feature->is_on_left_image)
    {
        features_left[landmark->id] = feature;
    }
    else
    {
        features_right[landmark->id] = feature;
    }
}

void Frame::RemoveFeature(visual::Feature::Ptr feature)
{
    assert(feature->is_on_left_image && id != feature->landmark.lock()->FirstFrame().lock()->id);
    int a = features_left.erase(feature->landmark.lock()->id);
}

} // namespace lvio_fusion
