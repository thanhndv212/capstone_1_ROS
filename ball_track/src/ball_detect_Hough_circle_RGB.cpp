#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

#include "core_msgs/ball_position.h"

#include <cv_bridge/cv_bridge.h>


int iMin_tracking_ball_size = 20; // This is the minimum tracking ball size, in pixels.
using namespace std;
using namespace cv;

// Declaration of trackbar functions to set HSV colorspace's parameters: In this section, we declare the functions that are displayed at the end on the multiple trackbars. We declare two sets of trackbar functions; one for the red ball, and one for the blue ball, Void functions are used so that they do not return any value. int is used when integer values are desired.
void on_low_h_thresh_trackbar_red(int, void *);
void on_high_h_thresh_trackbar_red(int, void *);
void on_low_h2_thresh_trackbar_red(int, void *);
void on_high_h2_thresh_trackbar_red(int, void *);
void on_low_s_thresh_trackbar_red(int, void *);
void on_high_s_thresh_trackbar_red(int, void *);
void on_low_v_thresh_trackbar_red(int, void *);
void on_high_v_thresh_trackbar_red(int, void *);

void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);
int low_r_r=0, low_g_r=0, low_b_r=112;
int high_r_r=80, high_g_r=115, high_b_r=200;

int low_r_b=106, low_g_b=0, low_b_b=0;
int high_r_b=190, high_g_b=111, high_b_b=255;

int diffdistance_r=180;
int diffdistance_b=180;
int upperT_r=180;
int upperT_b=180;
int centerT_r=20;
int centerT_b=20;

//Hough
void hough_circle_diff_distance_red(int, void *);
void hough_circle_diff_distance_blue(int, void *);
void hough_circle_upperT_red(int, void *);
void hough_circle_upperT_blue(int, void *);
void hough_circle_centerT_red(int, void *);
void hough_circle_centerT_blue(int, void *);

// Declaration of functions that changes data types: Here, we declare functions that change the type: from integer to string, and from float to string respectively.
string intToString(int n);
string floatToString(float f);

// Declaration of functions that changes int data to String
// Later in the code, we create a structuring element that is used to "dilate" and "erode" image. Here we declare it to not return any value using void.
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position: Using the pixel positions from the image observed by the camera, this function calculates the position of the ball, which is extremely important in our course goal. The vector function stores series of elements with the same variable name, in float data type.
vector<float> pixel2point(Point center, int radius);

// Declaration of trackbars function that set Canny edge's parameters: The Canny edge is a popular edge detecting algorithm, developed by John F. Canny. For the Sobel operations to be performed internally, we use kernel size of 3. We declare the canny edge trackbars for two sets; red and blue ball.
void on_canny_edge_trackbar_red(int, void *);
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;
void on_canny_edge_trackbar_blue(int, void *);
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

// Initialization of variable for dimension of the target: We set a float value for the radius of the desired targets, in this case the balls.
float fball_radius = 0.0734 ; // meter: The unit which is used in the initialization.

// Initialization of variable for camera calibration paramters: Like we did in our second class, we have to calibrate our main camera, and obtain the intrinsic and distortion parameters in order to undistort the images seen.
Mat distCoeffs;
float intrinsic_data[9] = {596.497705, 0, 318.023172, 0, 565.010517, 271.337191, 0, 0, 1};
float distortion_data[5] = {0.073823, -0.079246, 0.005644, -0.005499, 0};

// Initialization of variable for text drawing: The text which we see at the results is defined here
double fontScale = 2;
int thickness = 3;
String text ;

/////ROS publisher
ros::Publisher pub;



// Here, we start our main function.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_detect_node"); //init ros nodd
    ros::NodeHandle nh; //create node handler
    pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
    /////////////////////////////////////////////////////////////////////////

    core_msgs::ball_position msg;  //create a message for ball positions


    //////////////////////////////////////////////////////////////////
    Mat frame, rgb_frame, rgb_frame_red, rgb_frame_blue, rgb_frame_red_blur, rgb_frame_blue_blur, hsv_frame_red_canny, hsv_frame_blue_canny, result;
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);




    // Here, we start the video capturing function, with the argument being the camera being used. 0 indicates the default camera, and 1 indicates the additional camera. Also, we make the 6 windows which we see at the results.
    VideoCapture cap(1);
    namedWindow("Video Capture", WINDOW_NORMAL);
    namedWindow("Object Detection_RGB_Red", WINDOW_NORMAL);
    namedWindow("Object Detection_RGB_Blue", WINDOW_NORMAL);
    namedWindow("Canny Edge for Red Ball", WINDOW_NORMAL);
    namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);
    namedWindow("Result", WINDOW_NORMAL);

    moveWindow("Video Capture",              50, 0);
    moveWindow("Object Detection_RGB_Red",  50,370);
    moveWindow("Object Detection_RGB_Blue",470,370);
    moveWindow("Canny Edge for Red Ball",   50,730);
    moveWindow("Canny Edge for Blue Ball", 470,730);
    moveWindow("Result", 470, 0);


// Trackbars to set thresholds for HSV values : Red ball: In this part, we set the thresholds, in HSV color space values, for the red ball's trackbar. Since the red color has empty space in between, we need two sets of H values fro red ball.
    createTrackbar("Low R","Object Detection_RGB_Red", &low_r_r, 255, on_low_h_thresh_trackbar_red);
    createTrackbar("High R","Object Detection_RGB_Red", &high_r_r, 255, on_high_h_thresh_trackbar_red);
    createTrackbar("Low G","Object Detection_RGB_Red", &low_g_r, 255, on_low_s_thresh_trackbar_red);
    createTrackbar("High G","Object Detection_RGB_Red", &high_g_r, 255, on_high_s_thresh_trackbar_red);
    createTrackbar("Low B","Object Detection_RGB_Red", &low_b_r, 255, on_low_v_thresh_trackbar_red);
    createTrackbar("High B","Object Detection_RGB_Red", &high_b_r, 255, on_high_v_thresh_trackbar_red);

    // Trackbars to set thresholds for HSV values : Blue ball: In this part, we set the thresholds, in HSV color space, for the blue ball's trackbar.
    createTrackbar("Low R","Object Detection_RGB_Blue", &low_r_b, 255, on_low_h_thresh_trackbar_blue);
    createTrackbar("High R","Object Detection_RGB_Blue", &high_r_b, 255, on_high_h_thresh_trackbar_blue);
    createTrackbar("Low G","Object Detection_RGB_Blue", &low_g_b, 255, on_low_s_thresh_trackbar_blue);
    createTrackbar("High G","Object Detection_RGB_Blue", &high_g_b, 255, on_high_s_thresh_trackbar_blue);
    createTrackbar("Low B","Object Detection_RGB_Blue", &low_b_b, 255, on_low_v_thresh_trackbar_blue);
    createTrackbar("High B","Object Detection_RGB_Blue", &high_b_b, 255, on_high_v_thresh_trackbar_blue);
	//hough
    createTrackbar("diff_r","Object Detection_RGB_Red", &diffdistance_r, 180, hough_circle_diff_distance_red);
    createTrackbar("diff_b","Object Detection_RGB_Blue", &diffdistance_b, 180, hough_circle_diff_distance_blue);
    createTrackbar("upT_r","Object Detection_RGB_Red", &upperT_r, 180, hough_circle_upperT_red);
    createTrackbar("upT_b","Object Detection_RGB_Blue", &upperT_b, 180, hough_circle_upperT_blue);
    createTrackbar("cenT_r","Object Detection_RGB_Red", &centerT_r, 180, hough_circle_centerT_red);
    createTrackbar("cenT_b","Object Detection_RGB_Blue", &centerT_b, 180, hough_circle_centerT_blue);

    // Trackbar to set parameter for Canny Edge: In this part, we set the threshold for the Canny edge trackbar.
    createTrackbar("Min Threshold:","Canny Edge for Red Ball", &lowThreshold_r, 100, on_canny_edge_trackbar_red);
    createTrackbar("Min Threshold:","Canny Edge for Blue Ball", &lowThreshold_b, 100, on_canny_edge_trackbar_blue);

    while((char)waitKey(1)!='q'){
    cap>>frame;
    if(frame.empty())
        break;

    undistort(frame, calibrated_frame, intrinsic, distCoeffs);//Using the intrinsic and distortion data obtained from the camera calibration, we undistort the viewed image.

    result = calibrated_frame.clone();

    medianBlur(calibrated_frame, rgb_frame, 3);// Median blur function.

  //  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);// Color is converted from BGR color space to HSV color space.

    // Detect the object based on RGB and HSV Range Values: In the following lines, the camera uses the BGR to HSV converted colors to detect the different colored objects.

    inRange(rgb_frame,Scalar(low_r_r,low_g_r,low_b_r),Scalar(high_r_r,high_g_r,high_b_r),rgb_frame_red);

    inRange(rgb_frame,Scalar(low_r_b,low_g_b,low_b_b),Scalar(high_r_b,high_g_b,high_b_b),rgb_frame_blue);

    morphOps(rgb_frame_red);
    morphOps(rgb_frame_blue);

    // Gaussian blur function.
    GaussianBlur(rgb_frame_red, rgb_frame_red_blur, cv::Size(9, 9), 2, 2);
    GaussianBlur(rgb_frame_blue, rgb_frame_blue_blur, cv::Size(9, 9), 2, 2);

    //Canny edge function.
    Canny(rgb_frame_red_blur, hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
    Canny(rgb_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);



    //Find contour function.

    vector<Vec3f> circles_r;
    vector<Vec3f> circles_b;
     HoughCircles(hsv_frame_red_canny,circles_r,HOUGH_GRADIENT, 1, diffdistance_r, upperT_r, centerT_r, 5, 200); //proceed circle
     HoughCircles(hsv_frame_blue_canny,circles_b,HOUGH_GRADIENT, 1, diffdistance_b, upperT_b, centerT_b, 5, 200); //proceed circle
int count_r =0;
int count_b =0;
vector<float> ball_r_x;
vector<float> ball_r_y;
vector<float> ball_r_z;
vector<float> ball_b_x;
vector<float> ball_b_y;
vector<float> ball_b_z;


     for(int k=0;k<circles_b.size();k++){
       Vec3i c = circles_b[k];
       Point center = Point(c[0], c[1]);
       int radius = c[2];
       vector<float> params;

         params = pixel2point(center, radius);  //the information of k-th circle
        	 float cx=params[0];  //x position of k-th circle
         	float cy=params[1];  //y position
        	 float cz=params[2]; //radius
         // 원 출력을 위한 원 중심 생성
//         circle(frame,center,r,Scalar(255,0,0),10); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
	        count_b++;
            ball_b_x.push_back(cx);
            ball_b_y.push_back(cy);
            ball_b_z.push_back(cz);
	text = "Blue ball:"+floatToString(cx)+ ","+floatToString( cy)+ ","+floatToString(cz);
            putText(result, text, center,2,1,Scalar(255,0,0),2);
            circle( result, center, radius, Scalar(255,0,0), 2, 8, 0 );

}

     for(int k=0;k<circles_r.size();k++){
       Vec3i c = circles_r[k];
       Point center = Point(c[0], c[1]);
       int radius = c[2];
       vector<float> params;

         params = pixel2point(center, radius);  //the information of k-th circle
        	 float cx=params[0];  //x position of k-th circle
         	float cy=params[1];  //y position
        	 float cz=params[2]; //radius
         // 원 출력을 위한 원 중심 생성
	        count_r++;
            ball_r_x.push_back(cx);
            ball_r_y.push_back(cy);
            ball_r_z.push_back(cz);

	           text = "Red ball:"+floatToString(cx)+ ","+floatToString( cy)+ ","+floatToString(cz);
            putText(result, text, center,2,1,Scalar(0,0,255),2);
            circle( result, center, radius, Scalar(0,0,255), 2, 8, 0 );

}
     cout<<"circles_b.size="<<count_b<<endl;  //print the number of circles detected
     cout<<"circles_r.size="<<count_r<<endl;  //print the number of circles detected
	msg.size_r = count_r;
	msg.size_b = count_b;
	msg.img_x_r = ball_r_x;
	msg.img_y_r = ball_r_y;
	msg.img_z_r = ball_r_z;
	msg.img_x_b = ball_b_x;
	msg.img_y_b = ball_b_y;
	msg.img_z_b = ball_b_z;
    	pub.publish(msg);

    // Show the frames: Here, the 6 final widnows or frames are displayed for the user to see.
    imshow("Video Capture",calibrated_frame);
    imshow("Object Detection_RGB_Red",rgb_frame_red);
    imshow("Object Detection_RGB_Blue",rgb_frame_blue);
    imshow("Canny Edge for Red Ball", hsv_frame_red_canny);
    imshow("Canny Edge for Blue Ball", hsv_frame_blue_canny);
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

vector<float> pixel2point(Point center, int radius){
  vector<float> position;
    float x, y, u, v, Xc, Yc, Zc;
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

    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Zc);
    return position;
}
void on_low_h_thresh_trackbar_red(int, void *){
    setTrackbarPos("Low R","Object Detection_RGB_Red", low_r_r);
}
void on_high_h_thresh_trackbar_red(int, void *){
    setTrackbarPos("High R", "Object Detection_RGB_Red", high_r_r);
}
void on_low_s_thresh_trackbar_red(int, void *){
    setTrackbarPos("Low G","Object Detection_RGB_Red", low_g_r);
}
void on_high_s_thresh_trackbar_red(int, void *){
    setTrackbarPos("High G", "Object Detection_RGB_Red", high_g_r);
}
void on_low_v_thresh_trackbar_red(int, void *){
    setTrackbarPos("Low B","Object Detection_RGB_Red", low_b_r);
}
void on_high_v_thresh_trackbar_red(int, void *){
    setTrackbarPos("High B", "Object Detection_RGB_Red", high_b_r);
}

// Trackbar for image threshodling in HSV colorspace : Blue : The functions had been declared and created, and now they are positioned in the relevant result frames. In this case, in the blue ball's frames.
void on_low_h_thresh_trackbar_blue(int, void *){
    setTrackbarPos("Low R","Object Detection_RGB_Blue", low_r_b);
}
void on_high_h_thresh_trackbar_blue(int, void *){
    setTrackbarPos("High R", "Object Detection_RGB_Blue", high_r_b);
}
void on_low_s_thresh_trackbar_blue(int, void *){
    setTrackbarPos("Low G","Object Detection_RGB_Blue", low_g_b);
}
void on_high_s_thresh_trackbar_blue(int, void *){
    setTrackbarPos("High G", "Object Detection_RGB_Blue", high_g_b);
}
void on_low_v_thresh_trackbar_blue(int, void *){
    setTrackbarPos("Low B","Object Detection_RGB_Blue", low_b_b);
}
void on_high_v_thresh_trackbar_blue(int, void *){
    setTrackbarPos("High B", "Object Detection_RGB_Blue", high_b_b);
}
// Hough Circel
void hough_circle_diff_distance_red(int, void *){diffdistance_r=max(diffdistance_r,1);
    setTrackbarPos("diff_r","Object Detection_RGB_Red", diffdistance_r);
}
void hough_circle_diff_distance_blue(int, void *){diffdistance_b=max(diffdistance_b,1);
    setTrackbarPos("diff_b","Object Detection_RGB_Blue", diffdistance_b);
}
void hough_circle_upperT_red(int, void *){upperT_r=max(upperT_r,1);
    setTrackbarPos("upT_r","Object Detection_RGB_Red", upperT_r);
}
void hough_circle_upperT_blue(int, void *){upperT_b=max(upperT_b,1);
    setTrackbarPos("upT_b","Object Detection_RGB_Blue", upperT_b);
}
void hough_circle_centerT_red(int, void *){upperT_r=max(upperT_r,1);
    setTrackbarPos("cenT_r","Object Detection_RGB_Red", centerT_r);
}
void hough_circle_centerT_blue(int, void *){centerT_b=max(centerT_b,1);
    setTrackbarPos("cenT_b","Object Detection_RGB_Blue", centerT_b);
}


// Trackbar for Canny edge algorithm : The trackbars for the Canny edge window are positioned here, for both the red and blue balls.
void on_canny_edge_trackbar_red(int, void *){
setTrackbarPos("Min Threshold", "Canny Edge for Red Ball", lowThreshold_r);
}
void on_canny_edge_trackbar_blue(int, void *){
setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball", lowThreshold_b);
}
