#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include "core_msgs/roller_num.h"
#include <cv_bridge/cv_bridge.h>

#define INDEX_DEFAULT 1


using namespace std;
using namespace cv;

// Declaration of trackbar functions to set HSV colorspace's parameters: In this section, we declare the functions that are displayed at the end on the multiple trackbars. We declare two sets of trackbar functions; one for the red ball, and one for the blue ball, Void functions are used so that they do not return any value. int is used when integer values are desired.


int low_h_b=90, low_s_b=100, low_v_b=85;
int high_h_b=120, high_s_b=255, high_v_b=255;


// Declaration of functions that changes data types: Here, we declare functions that change the type: from integer to string, and from float to string respectively.
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
// Later in the code, we create a structuring element that is used to "dilate" and "erode" image. Here we declare it to not return any value using void.
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position: Using the pixel positions from the image observed by the camera, this function calculates the position of the ball, which is extremely important in our course goal. The vector function stores series of elements with the same variable name, in float data type.
vector<float> pixel2point(Point center, int radius);

// Declaration of trackbars function that set Canny edge's parameters: The Canny edge is a popular edge detecting algorithm, developed by John F. Canny. For the Sobel operations to be performed internally, we use kernel size of 3. We declare the canny edge trackbars for two sets; red and blue ball.
void on_canny_edge_trackbar_blue(int, void *);
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;
// Initialization of variable for dimension of the target: We set a float value for the radius of the desired targets, in this case the balls.
float fball_radius = 0.074 ; // meter: The unit which is used in the initialization.

// Initialization of variable for camera calibration paramters: Like we did in our second class, we have to calibrate our main camera, and obtain the intrinsic and distortion parameters in order to undistort the images seen.
Mat distCoeffs;
float intrinsic_data[9] = {646.25, 0, 321.3, 0, 650, 245.45, 0, 0, 1};
float distortion_data[5] = {0.09667, -0.26965, -0.000857, 0, 0};

// Initialization of variable for text drawing: The text which we see at the results is defined here
double fontScale = 2;
int thickness = 3;
String text ;

int iMin_tracking_ball_size = 240; // This is the minimum tracking ball size, in pixels.

/////ROS publisher
ros::Publisher pub;



void sigint_handler(int sig){
  exit(-1);
}

// Here, we start our main function.
int main(int argc, char **argv)
{
    int idx = (argc == 1)? INDEX_DEFAULT : (atoi(argv[1]));

    signal(SIGINT, sigint_handler);

    ros::init(argc, argv, "ball_counter"); //init ros nodd
    ros::NodeHandle nh; //create node handler
    pub = nh.advertise<core_msgs::roller_num>("/roller_num", 100); //setting publisher

    core_msgs::roller_num msg;  //create a message for ball positions

//declare frames using in image processing functions.
    Mat frame, bgr_frame, hsv_frame,  hsv_frame_blue, hsv_frame_blue_blur,  hsv_frame_blue_canny, result;
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);


    vector<Vec4i> hierarchy_b;

    vector<vector<Point> > contours_b;
    VideoCapture cap(idx);

    namedWindow("Result", WINDOW_NORMAL);

    moveWindow("Result", 470, 0);

    while((char)waitKey(1)!='q'){
    cap>>frame;
    if(frame.empty())
        break;

    undistort(frame, calibrated_frame, intrinsic, distCoeffs);//Using the intrinsic and distortion data obtained from the camera calibration, we undistort the viewed image.

    result = calibrated_frame.clone();

    medianBlur(calibrated_frame, calibrated_frame, 3);// Median blur function.

    cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);// Color is converted from BGR color space to HSV color space.

    // Detect the object based on RGB and HSV Range Values: In the following lines, the camera uses the BGR to HSV converted colors to detect the different colored objects.

    inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);

    morphOps(hsv_frame_blue);

    // Gaussian blur function.
    GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
    //Canny edge function.
    Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);

    //Find contour function.
    findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
    vector<vector<Point> > contours_b_poly( contours_b.size() );

    vector<Point2f>center_b( contours_b.size() );

    vector<float>radius_b( contours_b.size() );

         vector<Rect> boundRect( contours_b.size() );

         for( size_t i = 0; i < contours_b.size(); i++ )
 {
   approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
   minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );

 }

cv::Point_<int> center(230,250);
 float num =0;

for( size_t i = 0; i< contours_b.size(); i++ ){
  if(radius_b[i] > 191){ // for only larger than 191
      num=1;
            Scalar color = Scalar( 255, 0, 0);
            drawContours( result, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );


                            putText(result, "3 balls", center,2,3,Scalar(0,0,255),2);
                           text = num;


    }
  }


      cout <<num<<endl;

msg.size_b = num;
    pub.publish(msg);
    imshow("Result", result);


    }
    ros::spin();
    return 0;
}

string intToString(int n){stringstream s;
    s << n;
    return s.str();
}

string floatToString(float f){ostringstream buffer;
    buffer << f;
    return buffer.str();
}

void morphOps(Mat &thresh){//create structuring element that will be used to "dilate" and "erode" image.: These are morphological operations, they process input image based on shape and generate an output image. Erosion and dilation remove noise, isolate individual elements, join disparate elements, , and find intensity bumps or holes.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}
