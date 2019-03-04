#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>



int main(int argc, char** argv)
{
  cv::VideoCapture cap = cv::VideoCapture(0);
  cv::Mat frame;
  cv::Mat buffer;
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  sensor_msgs::ImagePtr msg;
  //cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    cap >> frame;
    //이미지 데이터의 크기를 줄이기 위해 원본 이미지보다 낮은 320*240 화질로 변경함
    cv::resize(frame, buffer, cv::Size(320, 240));
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", buffer).toImageMsg();
    //리사이즈한 이미지를 publish
    pub.publish(msg);
    //50ms 마다 publish(20Hz)
    cv::waitKey(50);

  }
  ros::spinOnce();
  loop_rate.sleep();
}
