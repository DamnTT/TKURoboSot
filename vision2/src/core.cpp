#include "core.hpp"

VisionCore::VisionCore()
  : it_(nh_)
{
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/camera/image_raw", 1,
    &VisionCore::imageCb, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);

  // cv::namedWindow(OPENCV_WINDOW);
}
  
VisionCore::~VisionCore()
{
  // cv::destroyWindow(OPENCV_WINDOW);
}

void VisionCore::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //// Update GUI Window
  // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  // cv::waitKey(3);
  
  cv::Mat output;
  cv::resize(cv_ptr->image, output, cv::Size(640, 480));

  // // Output modified video stream
  // image_pub_.publish(cv_ptr->toImageMsg());
  image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg());
}