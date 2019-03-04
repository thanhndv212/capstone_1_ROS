#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat buffer;
    cv::Mat frame;
    buffer =cv_bridge::toCvShare(msg, "bgr8")->image;
    // subscribe한 데이터를 640*480크기로 리사이즈
    cv::resize(buffer, frame, cv::Size(640, 480));
    // 필터할 (B,G,R)의 범위를 설정
      Mat Filterframe;
      Mat edges;
      // 필터할 (B,G,R)의 범위를 설정
      Scalar lb(0,100,200);
  		Scalar ub(150,250,300);
  		// frame 이미지 중, bound 내의 이미지를 Filterframe 이미지에 저장
  		inRange(frame,lb,ub,Filterframe);
      // Filterframe 이미지의 edge를 검출하여 edges 이미지에 저장
  		Canny(Filterframe,edges,50,200);
      // 검출된 원 정보가 저장될 vector를 설정
  		vector<Vec3f> circles;
      // edges 이미지에서 조건에 맞는 원을 검출하여 원정보를 circles에 저장
  		HoughCircles(edges,circles,HOUGH_GRADIENT, 1, 50, 200, 20, 3, 25);
      // 검출된 원 출력을 위하여 하나의 원 정보가 담길 vector인, params 생성
  		Vec3f params;
      // 원 중심과 반지름인 cx, cy, r 생성
  		double cx,cy,r;
      // 검출된 원의 수를 출력
  		cout<<"circles.size="<<circles.size()<<endl;
      // 각 원의 정보를 출력하고 그리는 for 문.
      // k=0부터 시작하여 한 번 루프를 돌 때 마다 1씩 증가하며, circles사이즈까지 for 문이 수행
      for(int k=0;k<circles.size();k++)
  		{
  			params = circles[k];
        cx=cvRound(params[0]);
        cy=cvRound(params[1]);
  		r=cvRound(params[2]);
        cout<<"circle("<<k<<"):cx="<<cx<<", cy="<<cy<<endl;
        // 원 출력을 위한 원 중심 생성
        Point center(cx,cy);
  			//frame 이미지에 center 중심으로, 반지름 r인 원을,Scalar(B,G,R) 색의 두께 10으로 그림
  			circle(frame,center,r,Scalar(0,0,255),10);
        cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));
        cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);
        cout<<"circle("<<k<<"):cx="<<cx<<", cy="<<cy<<endl;
      }
  		// frame 이미지를 "Result" 이름으로 출력
  	     imshow("view",frame);
         waitKey(50);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
}
