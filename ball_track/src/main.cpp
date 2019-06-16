#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include "core_msgs/ball_position.h"
#include <cv_bridge/cv_bridge.h>

#define INDEX_DEFAULT 0

using namespace std;
using namespace cv;

// Declaration of trackbar functions to set HSV colorspace's parameters: In this section, we declare the functions that are displayed at the end on the multiple trackbars. We declare two sets of trackbar functions; one for the red ball, and one for the blue ball, Void functions are used so that they do not return any value. int is used when integer values are desired.
int low_h2_r=155, high_h2_r=180;
int low_h_r=0, low_s_r=195, low_v_r=50;
int high_h_r=15, high_s_r=255, high_v_r=255;

int low_h_b=90, low_s_b=120, low_v_b=70;
int high_h_b=113, high_s_b=255, high_v_b=255;

int low_h_g=30, low_s_g=75, low_v_g=80;
int high_h_g=80, high_s_g=255, high_v_g=255;

// Declaration of functions that changes data types: Here, we declare functions that change the type: from integer to string, and from float to string respectively.
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
// Later in the code, we create a structuring element that is used to "dilate" and "erode" image. Here we declare it to not return any value using void.
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position: Using the pixel positions from the image observed by the camera, this function calculates the position of the ball, which is extremely important in our course goal. The vector function stores series of elements with the same variable name, in float data type.
vector<float> pixel2point(Point center, int radius);
//set default value
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;

// Initialization of variable for dimension of the target: We set a float value for the radius of the desired targets, in this case the balls.
float fball_radius = 0.074 ; // meter: The unit which is used in the initialization.

// Initialization of variable for camera calibration paramters: Like we did in our second class, we have to calibrate our main camera, and obtain the intrinsic and distortion parameters in order to undistort the images seen.
Mat distCoeffs;
float intrinsic_data[9] = {625.65, 0, 323.54, 0, 631.2, 254.9, 0, 0, 1};
float distortion_data[5] = {0.06263, -0.24675, 0.011155, 0.005235, 0};


// Initialization of variable for text drawing: The text which we see at the results is defined here
double fontScale = 2;
int thickness = 3;
String text ;

int iMin_tracking_ball_size = 25; // This is the minimum tracking ball size, in pixels.

/////ROS publisher
ros::Publisher pub;



void sigint_handler(int sig){
  exit(-1);
}

// Here, we start our main function.
int main(int argc, char **argv)
{ int idx = (argc == 1)? INDEX_DEFAULT : (atoi(argv[1]));
    signal(SIGINT, sigint_handler);

    ros::init(argc, argv, "ball_detect_node"); //init ros nodd
    ros::NodeHandle nh; //create node handler
    pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher

    core_msgs::ball_position msg;
//set frames using in image manipulation functions
    Mat frame, bgr_frame, hsv_frame, hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue,hsv_frame_green, hsv_frame_red_blur, hsv_frame_blue_blur, hsv_frame_green_blur, hsv_frame_red_canny, hsv_frame_blue_canny,hsv_frame_green_canny, result;
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);

//declare vectors using in findcontours function
    vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;
    vector<Vec4i> hierarchy_g;

    vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;
    vector<vector<Point> > contours_g;

    // Here, we start the video capturing function, with the argument being the camera being used. 0 indicates the default camera, and 1 indicates the additional camera. Also, we make the 6 windows which we see at the results.
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

    inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);

    inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);

    inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);

    inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);

    addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

    morphOps(hsv_frame_red);
    morphOps(hsv_frame_blue);
    morphOps(hsv_frame_green);

    // Gaussian blur function.
    GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
    GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
    GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9, 9), 2, 2);
    //Canny edge function.
    Canny(hsv_frame_red_blur, hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
    Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);
    Canny(hsv_frame_green_blur, hsv_frame_green_canny, lowThreshold_g, lowThreshold_g*ratio_g, kernel_size_g);

    //Find contour function.
    findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(hsv_frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
    //resize to number of detected balls
    vector<vector<Point> > contours_r_poly( contours_r.size() );
    vector<vector<Point> > contours_b_poly( contours_b.size() );
    vector<vector<Point> > contours_g_poly( contours_g.size() );

    vector<Point2f>center_r( contours_r.size() );
    vector<Point2f>center_b( contours_b.size() );
    vector<Point2f>center_g( contours_g.size() );

    vector<float>radius_r( contours_r.size() );
    vector<float>radius_b( contours_b.size() );
    vector<float>radius_g( contours_g.size() );

//Minenclosing circle function.
        for( size_t i = 0; i < contours_b.size(); i++ ){
            approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
            minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
        }
        for( size_t i = 0; i < contours_r.size(); i++ ){
            approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
            minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
        }
    for( size_t i = 0; i < contours_g.size(); i++ ){approxPolyDP( contours_g[i], contours_g_poly[i], 3, true );
        minEnclosingCircle( contours_g_poly[i], center_g[i], radius_g[i] );}

int count_r =0;
int count_b =0;
int count_g=0;
vector<float> ball_r_x, ball_r_y, ball_r_z, ball_r_radius;
vector<float> ball_b_x, ball_b_y, ball_b_z, ball_b_radius;
vector<float> ball_g_x, ball_g_y, ball_g_z, ball_g_radius;
//draw circles in screen and processing
    for( size_t i = 0; i< contours_b.size(); i++ ){
      if((radius_b[i] > iMin_tracking_ball_size) && (15<center_b[i].x)&&(center_b[i].x < 605)){ //for erasing errors using min radius and side of the screen
            vector<float> ball_position_b;
            ball_position_b = pixel2point(center_b[i], radius_b[i]);
	      
            float dis = ball_position_b[2];
            float pixel = 0.0002*pow(radius_b[i],2)-0.0362*radius_b[i]+1.766;//추세선 계산
		// error가 30cm 이하일 때
	      	if ( (pixel - dis) <0.3){

		            ball_b_x.push_back(ball_position_b[0]);
                ball_b_y.push_back(ball_position_b[1]);
                ball_b_z.push_back(ball_position_b[2]);
                ball_b_radius.push_back(ball_position_b[3]);
		              count_b++;
		//같은 공 여러개로 보이는 것 제거
                  float x1, y1, x2, y2;
                  x1 =center_b[i].x;
		              x2 =center_b[i+1].x;
                  y1=center_b[i].y;
		              y2= center_b[i+1].y;
                  float diff = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
                  float l = radius_b[i];

                Scalar color = Scalar( 255, 0, 0);
                drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                text = "Solution1";
                putText(result, text, center_b[i],2,1,Scalar(0,255,0),2);
                circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
		              cout<<"blue"<<i<<"\t"<<dis<<endl;
                if (abs(l)>diff)i++;
		}
        }
      }

for( size_t i = 0; i< contours_r.size(); i++ ){
    if((radius_r[i] > iMin_tracking_ball_size)&& (15<center_r[i].x)&&(center_r[i].x < 605)){
          vector<float> ball_position_r;
          ball_position_r = pixel2point(center_r[i], radius_r[i]);
          float dis = ball_position_r[2];
          float pixel = 0.0002*pow(radius_r[i],2)-0.0362*radius_r[i]+1.766;

            ball_r_x.push_back(ball_position_r[0]);
            ball_r_y.push_back(ball_position_r[1]);
            ball_r_z.push_back(ball_position_r[2]);
            ball_r_radius.push_back(ball_position_r[3]);
            float x1, y1, x2, y2;
            x1 =center_r[i].x;
            x2 =center_r[i+1].x;
            y1=center_r[i].y;
            y2= center_r[i+1].y;
            float diff = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
            float l = radius_r[i];
              count_r++;
                Scalar color = Scalar( 0, 0, 255);
                drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                text = "Solution1";
                cout<<"red"<<i<<"\t"<<dis<<endl;

                putText(result, text, center_r[i],2,1,Scalar(0,255,0),2);
                circle( result, center_r[i], (int)radius_r[i], color, 2, 8, 0 );
                if (abs(l)>diff)i++;
              }}

    for( size_t i = 0; i< contours_g.size(); i++ ){
      if (radius_g[i] > iMin_tracking_ball_size){Scalar color = Scalar( 0, 255,0 );
            drawContours( hsv_frame_green_canny, contours_g_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );

            vector<float> ball_position_g;
            ball_position_g = pixel2point(center_g[i], radius_g[i]);
            float isx = ball_position_g[0];
            float isy = ball_position_g[1];
            float isz = ball_position_g[2];
            float radi = ball_position_g[3];
            string sx = floatToString(isx);
            string sy = floatToString(isy);
            string sz = floatToString(isz);
            float dis = ball_position_g[2];
            float pixel = 0.0002*pow(radius_g[i],2)-0.0362*radius_g[i]+1.766;
	if ( (pixel - dis) <0.3){
            ball_g_x.push_back(isx);
            ball_g_y.push_back(isy);
            ball_g_z.push_back(isz);
            ball_g_radius.push_back(radi);
               count_g++;
       
            text = "Green ball:" + sx + "," + sy + "," + sz;
            putText(result, text, center_g[i],2,1,Scalar(0,255,0),2);
            circle( result, center_g[i], (int)radius_g[i], color, 2, 8, 0 );}
		 float x1, y1, x2, y2;
    x1 =center_g[i].x;
            x2 =center_g[i+1].x;
            y1=center_g[i].y;
            y2= center_g[i+1].y;
		 float diff =sqrt( pow((x1-x2),2)+pow((y1-y2),2));
               float l = radius_g[i];
            if (abs(l)>diff)i++;}
						
        }
//msg에 계산된 값들 넣어줌
msg.size_b = count_b;
msg.img_x_b = ball_b_x;
msg.img_y_b = ball_b_y;
msg.img_z_b = ball_b_z;
msg.size_r = count_r;
msg.img_x_r = ball_r_x;
msg.img_y_r = ball_r_y;
msg.img_z_r = ball_r_z;
msg.size_g = count_g;
msg.img_x_g = ball_g_x;
msg.img_y_g = ball_g_y;
msg.img_z_g = ball_g_z;
//publish
    pub.publish(msg);
cout <<count_b<<count_r<<endl;
    // Show the frames: Here, the 6 final widnows or frames are displayed for the user to see.
    imshow("Result", result);



    }
    ros::spin();
    return 0;
}
//chage format int to string
string intToString(int n){stringstream s;
    s << n;
    return s.str();
}
//change format float to string
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
//calculate real distance using calibration.
vector<float> pixel2point(Point center, int radius){vector<float> position;
    float x, y, u, v, Xc, Yc, Zc,Pc;
    x = center.x;//.x;// .at(0);
    y = center.y;//.y;//
    u = (x-intrinsic_data[2])/intrinsic_data[0];
    v = (y-intrinsic_data[5])/intrinsic_data[4];

    Zc = (intrinsic_data[0]*fball_radius)/(2*(float)radius) ;
    Xc = u*Zc ;
    Yc = v*Zc ;
    Xc = roundf(Xc * 1000) / 1000;
    Yc = roundf(Yc * 1000) / 1000;
    Zc = roundf(Zc * 1000) / 1000;
    Pc = 0.000006*pow(y,2)-0.005*y+1.2548;
    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Pc);
    return position;
}
