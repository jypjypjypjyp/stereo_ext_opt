
#include "lvio_fusion/visual/landmark.h"
#include "lvio_fusion/frame.h"
#include "lvio_fusion/utility.h"
#include "lvio_fusion/visual/camera.h"

namespace lvio_fusion
{

namespace visual
{
unsigned long Landmark::current_landmark_id = 0;

Vector3d Landmark::ToWorld()
{
    Vector3d pb = Camera::Get(1)->Pixel2Robot(cv2eigen(first_observation->keypoint.pt), 1 / inv_depth);
    return Camera::Get()->Robot2World(pb, FirstFrame().lock()->pose);
}

visual::Landmark::Ptr Landmark::Create(double inv_depth)
{
    visual::Landmark::Ptr new_point(new Landmark);
    new_point->inv_depth = inv_depth;
    return new_point;
}

void Landmark::Clear()
{
}

std::weak_ptr<Frame> Landmark::FirstFrame()
{
    auto frame = first_observation->frame;
    assert(!frame.expired());
    return frame;
}

std::weak_ptr<Frame> Landmark::LastFrame()
{
    auto frame = (--observations.end())->second->frame;
    assert(!frame.expired());
    return frame;
}

void Landmark::AddObservation(visual::Feature::Ptr feature)
{
    assert(feature->landmark.lock()->id == id);
    if (feature->is_on_left_image)
    {
        observations[feature->frame.lock()->id] = feature;
    }
    else
    {
        assert(feature->frame.lock()->id == observations.begin()->first);
        first_observation = feature;
    }
}

void Landmark::RemoveObservation(visual::Feature::Ptr feature)
{
    assert(feature->is_on_left_image && feature != observations.begin()->second);
    observations.erase(feature->frame.lock()->id);
}
} // namespace visual

} // namespace lvio_fusion
