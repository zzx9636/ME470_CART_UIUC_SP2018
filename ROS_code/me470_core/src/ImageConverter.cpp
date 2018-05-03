#include "../include/ImageConverter.h"

static const std::string OPENCV_WINDOW = "Image window";

ImageConverter::ImageConverter():it_(nh_)
{
// Subscrive to input video feed and publish output video feed
image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
    &ImageConverter::ros2cv, this);

}

void ImageConverter::ros2cv(const sensor_msgs::ImageConstPtr& msg)
{
cv_bridge::CvImagePtr cv_ptr;
try
{
    cv_ptr = cv_bridge::toCvCopy(msg);
}
catch (cv_bridge::Exception& e)
{
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
}

cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);

current_dis=Detector.detection_distance(gray,3);
}