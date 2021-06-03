#include "lvio_fusion/estimator.h"
#include "lvio_fusion/config.h"
#include "lvio_fusion/frame.h"
#include "lvio_fusion/visual/camera.h"

#include <opencv2/core/eigen.hpp>
#include <sys/sysinfo.h>

const double epsilon = 1e-3;
const int num_threads = std::min(8, std::max(1, (int)(0.75 * get_nprocs())));
const double max_speed = 40;

namespace lvio_fusion
{
double Camera::baseline = 0;
std::vector<Camera::Ptr> Camera::devices_;

Estimator::Estimator(std::string &config_path) : config_file_path_(config_path) {}

bool Estimator::Init()
{
    LOG(INFO) << "System info:\n\tepsilon: " << epsilon << "\n\tnum_threads: " << num_threads;

    // read from config file
    if (!Config::SetParameterFile(config_file_path_))
    {
        return false;
    }

    // read camera intrinsics and extrinsics
    bool undistort = Config::Get<int>("undistort");
    cv::Mat cv_body_to_cam0 = Config::Get<cv::Mat>("body_to_cam0");
    cv::Mat cv_body_to_cam1 = Config::Get<cv::Mat>("body_to_cam1");
    Matrix4d body_to_cam0, body_to_cam1;
    cv::cv2eigen(cv_body_to_cam0, body_to_cam0);
    cv::cv2eigen(cv_body_to_cam1, body_to_cam1);
    // first camera
    Matrix3d R_body_to_cam0(body_to_cam0.block(0, 0, 3, 3));
    Quaterniond q_body_to_cam0(R_body_to_cam0);
    Vector3d t_body_to_cam0(0, 0, 0);
    t_body_to_cam0 << body_to_cam0(0, 3), body_to_cam0(1, 3), body_to_cam0(2, 3);
    if (undistort)
    {
        Camera::Create(Config::Get<double>("camera0.fx"),
                       Config::Get<double>("camera0.fy"),
                       Config::Get<double>("camera0.cx"),
                       Config::Get<double>("camera0.cy"),
                       Config::Get<double>("camera0.k1"),
                       Config::Get<double>("camera0.k2"),
                       Config::Get<double>("camera0.p1"),
                       Config::Get<double>("camera0.p2"),
                       SE3d(q_body_to_cam0, t_body_to_cam0));
    }
    else
    {
        Camera::Create(Config::Get<double>("camera0.fx"),
                       Config::Get<double>("camera0.fy"),
                       Config::Get<double>("camera0.cx"),
                       Config::Get<double>("camera0.cy"),
                       SE3d(q_body_to_cam0, t_body_to_cam0));
    }
    // second camera
    Matrix3d R_body_to_cam1(body_to_cam1.block(0, 0, 3, 3));
    Quaterniond q_body_to_cam1(R_body_to_cam1);
    Vector3d t_body_to_cam1(0, 0, 0);
    t_body_to_cam1 << body_to_cam1(0, 3), body_to_cam1(1, 3), body_to_cam1(2, 3);
    if (undistort)
    {
        Camera::Create(Config::Get<double>("camera1.fx"),
                       Config::Get<double>("camera1.fy"),
                       Config::Get<double>("camera1.cx"),
                       Config::Get<double>("camera1.cy"),
                       Config::Get<double>("camera1.k1"),
                       Config::Get<double>("camera1.k2"),
                       Config::Get<double>("camera1.p1"),
                       Config::Get<double>("camera1.p2"),
                       SE3d(q_body_to_cam1, t_body_to_cam1));
    }
    else
    {
        Camera::Create(Config::Get<double>("camera1.fx"),
                       Config::Get<double>("camera1.fy"),
                       Config::Get<double>("camera1.cx"),
                       Config::Get<double>("camera1.cy"),
                       SE3d(q_body_to_cam1, t_body_to_cam1));
    }
    Camera::baseline = (t_body_to_cam0 - t_body_to_cam1).norm();

    // create components and links
    frontend = Frontend::Ptr(new Frontend);

    return true;
}

void Estimator::InputImage(double time, cv::Mat &left_image, cv::Mat &right_image)
{
    Frame::Ptr new_frame = Frame::Create();
    new_frame->time = time;
    cv::undistort(left_image, new_frame->image_left, Camera::Get(0)->K, Camera::Get(0)->D);
    cv::undistort(right_image, new_frame->image_right, Camera::Get(1)->K, Camera::Get(1)->D);

    bool success = frontend->AddFrame(new_frame);
}

} // namespace lvio_fusion
