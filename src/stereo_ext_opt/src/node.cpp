#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "lvio_fusion/common.h"
#include "lvio_fusion/estimator.h"
#include "lvio_fusion/utility.h"
#include "parameters.h"

using namespace lvio_fusion;
using namespace std;

Estimator::Ptr estimator;

ros::Subscriber sub_img0, sub_img1;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
mutex m_img_buf;

// requisite topic
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_img_buf.lock();
    img0_buf.push(img_msg);
    m_img_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_img_buf.lock();
    img1_buf.push(img_msg);
    m_img_buf.unlock();
}

cv::Mat get_image_from_msg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    cv::Mat image;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        image = ptr->image.clone();
    }
    else if (img_msg->encoding == "bgr8")
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(ptr->image, image, cv::COLOR_BGR2GRAY);
    }
    else
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        image = ptr->image.clone();
    }

    cv::equalizeHist(image, image);
    return image;
}

// extract images with same timestamp from two topics
void sync_process()
{
    int n = 0;
    while (ros::ok())
    {
        cv::Mat image0, image1;
        std_msgs::Header header;
        double time = 0;
        if (!img0_buf.empty() && !img1_buf.empty())
        {
            m_img_buf.lock();
            double time0 = img0_buf.front()->header.stamp.toSec();
            double time1 = img1_buf.front()->header.stamp.toSec();
            if (time0 < time1 - 5 * epsilon)
            {
                img0_buf.pop();
                printf("throw img0\n");
                m_img_buf.unlock();
            }
            else if (time0 > time1 + 5 * epsilon)
            {
                img1_buf.pop();
                printf("throw img1\n");
                m_img_buf.unlock();
            }
            else
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image0 = get_image_from_msg(img0_buf.front());
                image1 = get_image_from_msg(img1_buf.front());
                img0_buf.pop();
                img1_buf.pop();
                m_img_buf.unlock();
                estimator->InputImage(time, image0, image1);
            }
        }
        chrono::milliseconds dura(2);
        this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_ext_opt");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    string config_file = "read_config_path_is_not_correct";

    if (argc > 1)
    {
        if (argc != 2)
        {
            printf("Please intput: rosrun lvio_fusion_node lvio_fusion_node [config file] \n"
                   "for example: rosrun lvio_fusion_node lvio_fusion_node "
                   "~/Projects/lvio-fusion/src/lvio_fusion_node/config/kitti.yaml \n");
            return 1;
        }

        config_file = argv[1];
    }
    else
    {
        if (!n.getParam("config_file", config_file))
        {
            ROS_ERROR("Error: %s", config_file.c_str());
            return 1;
        }
        ROS_INFO("Load config_file: %s", config_file.c_str());
    }
    read_parameters(config_file);
    estimator = Estimator::Ptr(new Estimator(config_file));
    assert(estimator->Init() == true);
    ROS_WARN("Waiting for images...");

    cout << "image0:" << IMAGE0_TOPIC << endl;
    sub_img0 = n.subscribe(IMAGE0_TOPIC, 10, img0_callback);
    cout << "image1:" << IMAGE1_TOPIC << endl;
    sub_img1 = n.subscribe(IMAGE1_TOPIC, 10, img1_callback);
    thread sync_thread{sync_process};
    ros::spin();
    return 0;
}
