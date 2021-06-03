#ifndef lvio_fusion_VISUAL_ERROR_H
#define lvio_fusion_VISUAL_ERROR_H

#include "lvio_fusion/ceres/base.hpp"
#include "lvio_fusion/visual/camera.h"

namespace lvio_fusion
{

template <typename T>
inline void Reprojection(const T *pw, const T *Twc, Camera::Ptr camera, T *result)
{
    T Twc_i[7], pc[3];
    ceres::SE3Inverse(Twc, Twc_i);
    ceres::SE3TransformPoint(Twc_i, pw, pc);
    T xp = pc[0] / pc[2];
    T yp = pc[1] / pc[2];
    result[0] = camera->fx * xp + camera->cx;
    result[1] = camera->fy * yp + camera->cy;
}

template <typename T>
inline void Pixel2Robot(const T *ob, const T *inv_d, Camera::Ptr camera, T *result)
{
    T d = T(1) / inv_d[0];
    T ps[3] = {T((ob[0] - camera->cx) / camera->fx) * d, T((ob[1] - camera->cy) / camera->fy) * d, d};
    T e[7];
    ceres::Cast(camera->extrinsic.data(), SE3d::num_parameters, e);
    ceres::SE3TransformPoint(e, ps, result);
}

template <typename T>
inline void Robot2Pixel(const T *pb, Camera::Ptr camera, T *result)
{
    T e[7], e_i[7], pc[3];
    ceres::Cast(camera->extrinsic.data(), SE3d::num_parameters, e);
    ceres::SE3Inverse(e, e_i);
    ceres::SE3TransformPoint(e_i, pb, pc);
    T xp = pc[0] / pc[2];
    T yp = pc[1] / pc[2];
    result[0] = camera->fx * xp + camera->cx;
    result[1] = camera->fy * yp + camera->cy;
}

class ExtrinctError
{
public:
    ExtrinctError(Vector3d A, Vector3d B, Vector2d ob, Vector3d t, Camera::Ptr left, Camera::Ptr right)
        : A_(A), B_(B), ob_(ob), t_(t), left(left), right(right) {}

    template <typename T>
    bool operator()(const T *R, T *residuals) const
    {
        T a[2], b[2], c[2] = {T(ob_.x()), T(ob_.y())};
        T A[3] = {T(A_.x()), T(A_.y()), T(A_.z())}, B[3] = {T(B_.x()), T(B_.y()), T(B_.z())};
        T Ex[7] = {R[0], R[1], R[2], R[3], T(t_[0]), T(t_[1]), T(t_[2])};
        Reprojection(A, Ex, right, a);
        Reprojection(B, Ex, right, b);
        T AB[2] = {b[0] - a[0], b[1] - a[1]},
          AC[2] = {c[0] - a[0], c[1] - a[1]};
        T ABAC = abs(AB[0] * AC[1] - AB[1] * AC[0]);
        T n = sqrt(AB[0] * AB[0] + AB[1] * AB[1]);
        residuals[0] = ABAC / n;
        return true;
    }

    static ceres::CostFunction *Create(Vector3d A, Vector3d B, Vector2d ob, Vector3d t, Camera::Ptr left, Camera::Ptr right)
    {
        return (new ceres::AutoDiffCostFunction<ExtrinctError, 1, 4>(
            new ExtrinctError(A, B, ob, t, left, right)));
    }

private:
    Vector3d A_, B_, t_;
    Vector2d ob_;
    Camera::Ptr left, right;
};

} // namespace lvio_fusion

#endif // lvio_fusion_VISUAL_ERROR_H
