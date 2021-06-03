#include "lvio_fusion/frontend.h"
#include "lvio_fusion/ceres/visual_error.hpp"
#include "lvio_fusion/utility.h"
#include "lvio_fusion/visual/camera.h"

namespace lvio_fusion
{
cv::Mat img_track;
void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                      Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void Frontend::EstimateD(Points &ll, Points &rr, std::vector<double> &dd)
{
    Matrix<double, 3, 4> P = Camera::Get()->extrinsic.inverse().matrix3x4(),
                         P1 = Camera::Get(1)->extrinsic.inverse().matrix3x4();

    int front_count = 0;
    for (int i = 0; i < ll.size(); i++)
    {
        Vector3d p3d;
        auto a = Vector2d(ll[i].x, ll[i].y);
        auto b = Vector2d(rr[i].x, rr[i].y);
        Vector2d aa = Camera::Get()->Pixel2Sensor(a).head(2);
        Vector2d bb = Camera::Get(1)->Pixel2Sensor(b).head(2);
        triangulatePoint(P, P1, aa, bb, p3d);
        Eigen::Vector3d localPoint;
        localPoint = P.leftCols<3>() * p3d + P.rightCols<1>();
        dd.push_back(localPoint.z());
    }
}

void Frontend::OptimizeEx(Points &ll, Points &rr)
{
    ceres::Problem problem;
    double *para = Camera::Get(1)->extrinsic.data();
    problem.AddParameterBlock(para, SO3d::num_parameters, new ceres::EigenQuaternionParameterization());
    for (int i = 0; i < ll.size(); i++)
    {
        Vector2d l = cv2eigen(ll[i]), r = cv2eigen(rr[i]);
        Vector3d A = Camera::Get()->Pixel2Robot(l, 1), B = Camera::Get()->Pixel2Robot(l, 10);
        Vector2d a = Camera::Get(1)->Robot2Pixel(A), b = Camera::Get(1)->Robot2Pixel(B);
        cv::arrowedLine(img_track, eigen2cv(a), eigen2cv(b), cv::Scalar(255, 0, 0), 1, 8, 0, 0.2);
        ceres::CostFunction *cost_function = ExtrinctError::Create(A, B, r, Camera::Get(1)->extrinsic.translation(), Camera::Get(0), Camera::Get(1));
        problem.AddResidualBlock(cost_function, NULL, para);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = num_threads;
    options.max_num_iterations = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}


bool Frontend::AddFrame(Frame::Ptr frame)
{
    std::unique_lock<std::mutex> lock(mutex);
    current_frame = frame;
    cv::cvtColor(current_frame->image_right, img_track, cv::COLOR_GRAY2RGB);
    Points kps_left, kps_right, ll, rr;
    std::vector<uchar> status;
    cv::goodFeaturesToTrack(frame->image_left, kps_left, 500, 0.01, 10);
    kps_right = kps_left;
    optical_flow(frame->image_left, frame->image_right, kps_left, kps_right, status);
    for (int i = 0; i < kps_left.size(); i++)
    {
        if (status[i])
        {
            ll.push_back(kps_left[i]);
            rr.push_back(kps_right[i]);
            cv::arrowedLine(img_track, kps_left[i], kps_right[i], cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }
    OptimizeEx(ll, rr);
    LOG(INFO) << Camera::Get(1)->extrinsic.matrix3x4();
    std::vector<double> dd;
    EstimateD(ll, rr, dd);
    for (int i = 0; i < dd.size(); i++)
    {
        if (dd[i] > 0)
        {
            auto pb = Camera::Get()->Pixel2Robot(cv2eigen(ll[i]), dd[i]);
            auto pixel = eigen2cv(Camera::Get(1)->Robot2Pixel(pb));
            cv::arrowedLine(img_track, ll[i], pixel, cv::Scalar(0, 0, 255), 1, 8, 0, 0.2);
        }
    }
    cv::imshow("tracking", img_track);
    cv::waitKey(1);
    return true;
}

} // namespace lvio_fusion