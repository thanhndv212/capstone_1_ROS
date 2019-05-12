#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include "core_msgs/ball_position_top.h"
#include <cv_bridge/cv_bridge.h>



using namespace std;
using namespace cv;

// Declaration of trackbar functions to set HSV colorspace's parameters: In this section, we declare the functions that are displayed at the end on the multiple trackbars. We declare two sets of trackbar functions; one for the red ball, and one for the blue ball, Void functions are used so that they do not return any value. int is used when integer values are desired.
int low_h2_r=155, high_h2_r=180;
int low_h_r=0, low_s_r=200, low_v_r=100;
int high_h_r=15, high_s_r=255, high_v_r=255;

int low_h_b=90, low_s_b=170, low_v_b=90;
int high_h_b=113, high_s_b=255, high_v_b=255;
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
float fball_radius = 0.075 ; // meter: The unit which is used in the initialization.

// Initialization of variable for camera calibration paramters: Like we did in our second class, we have to calibrate our main camera, and obtain the intrinsic and distortion parameters in order to undistort the images seen.
Mat distCoeffs;
float intrinsic_data[9] = {646.25, 0, 321.3, 0, 650, 245.45, 0, 0, 1};
float distortion_data[5] = {0.09667, -0.26965, -0.000857, 0, 0};

// Initialization of variable for text drawing: The text which we see at the results is defined here
double fontScale = 2;
int thickness = 3;
String text ;

int iMin_tracking_ball_size = 12; // This is the minimum tracking ball size, in pixels.

/////ROS publisher
ros::Publisher pub;



void sigint_handler(int sig){
  exit(-1);
}

// Here, we start our main function.
int main(int argc, char **argv)
{
    signal(SIGINT, sigint_handler);

    ros::init(argc, argv, "ball_detect_node"); //init ros nodd
    ros::NodeHandle nh; //create node handler
    pub = nh.advertise<core_msgs::ball_position_top>("/position_top", 100); //setting publisher

    /////////////////////////////////////////////////////////////////////////

    core_msgs::ball_position_top msg;  //create a message for ball positions


    //////////////////////////////////////////////////////////////////
    Mat frame, bgr_frame, hsv_frame, hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue, hsv_frame_red_blur, hsv_frame_blue_blur, hsv_frame_red_canny, hsv_frame_blue_canny, result;
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);

    vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;
    vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;

    // Here, we start the video capturing function, with the argument being the camera being used. 0 indicates the default camera, and 1 indicates the additional camera. Also, we make the 6 windows which we see at the results.
    VideoCapture cap(1);
    namedWindow("Video Capture", WINDOW_NORMAL);
    namedWindow("Object Detection_HSV_Red", WINDOW_NORMAL);
    namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
    namedWindow("Canny Edge for Red Ball", WINDOW_NORMAL);
    namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);
    namedWindow("Result", WINDOW_NORMAL);

    moveWindow("Video Capture",              50, 0);
    moveWindow("Object Detection_HSV_Red",  50,370);
    moveWindow("Object Detection_HSV_Blue",470,370);
    moveWindow("Canny Edge for Red Ball",   50,730);
    moveWindow("Canny Edge for Blue Ball", 470,730);
    moveWindow("Result", 470, 0);



// Trackbars to set thresholds for HSV values : Red ball: In this part, we set the thresholds, in HSV color space values, for the red ball's trackbar. Since the red color has empty space in between, we need two sets of H values fro red ball.
    createTrackbar("Low H","Object Detection_HSV_Red", &low_h_r, 180, on_low_h_thresh_trackbar_red);
    createTrackbar("High H","Object Detection_HSV_Red", &high_h_r, 180, on_high_h_thresh_trackbar_red);
    createTrackbar("Low H2","Object Detection_HSV_Red", &low_h2_r, 180, on_low_h2_thresh_trackbar_red);
    createTrackbar("High H2","Object Detection_HSV_Red", &high_h2_r, 180, on_high_h2_thresh_trackbar_red);
    createTrackbar("Low S","Object Detection_HSV_Red", &low_s_r, 255, on_low_s_thresh_trackbar_red);
    createTrackbar("High S","Object Detection_HSV_Red", &high_s_r, 255, on_high_s_thresh_trackbar_red);
    createTrackbar("Low V","Object Detection_HSV_Red", &low_v_r, 255, on_low_v_thresh_trackbar_red);
    createTrackbar("High V","Object Detection_HSV_Red", &high_v_r, 255, on_high_v_thresh_trackbar_red);

    // Trackbars to set thresholds for HSV values : Blue ball: In this part, we set the thresholds, in HSV color space, for the blue ball's trackbar.
    createTrackbar("Low H","Object Detection_HSV_Blue", &low_h_b, 180, on_low_h_thresh_trackbar_blue);
    createTrackbar("High H","Object Detection_HSV_Blue", &high_h_b, 180, on_high_h_thresh_trackbar_blue);
    createTrackbar("Low S","Object Detection_HSV_Blue", &low_s_b, 255, on_low_s_thresh_trackbar_blue);
    createTrackbar("High S","Object Detection_HSV_Blue", &high_s_b, 255, on_high_s_thresh_trackbar_blue);
    createTrackbar("Low V","Object Detection_HSV_Blue", &low_v_b, 255, on_low_v_thresh_trackbar_blue);
    createTrackbar("High V","Object Detection_HSV_Blue", &high_v_b, 255, on_high_v_thresh_trackbar_blue);

    // Trackbar to set parameter for Canny Edge: In this part, we set the threshold for the Canny edge trackbar.
    createTrackbar("Min Threshold:","Canny Edge for Red Ball", &lowThreshold_r, 100, on_canny_edge_trackbar_red);
    createTrackbar("Min Threshold:","Canny Edge for Blue Ball", &lowThreshold_b, 100, on_canny_edge_trackbar_blue);


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

    addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

    morphOps(hsv_frame_red);
    morphOps(hsv_frame_blue);

    // Gaussian blur function.
    GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
    GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);

    //Canny edge function.
    Canny(hsv_frame_red_blur, hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
    Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);

    //Find contour function.
    findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > contours_r_poly( contours_r.size() );
    vector<vector<Point> > contours_b_poly( contours_b.size() );
    vector<Point2f>center_r( contours_r.size() );
    vector<Point2f>center_b( contours_b.size() );
    vector<float>radius_r( contours_r.size() );
    vector<float>radius_b( contours_b.size() );
////////////////////////////////////////////////
    //msg_b.size =contours_b.size();//adding numbers of blue balls to /ball_position messagem
    //msg_b.img_x.resize(contours_b.size());
    //msg_b.img_y.resize(contours_b.size());
    /////////////////////////////////////
    for( size_t i = 0; i < contours_r.size(); i++ ){approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
        minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
    }

    for( size_t i = 0; i < contours_b.size(); i++ ){approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
        minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
    }
int count_r =0;
int count_b =0;
vector<float> ball_r_x;
vector<float> ball_r_y;
vector<float> ball_r_z;
vector<float> ball_b_x;
vector<float> ball_b_y;
vector<float> ball_b_z, ball_b_radius , ball_r_radius;

    for( size_t i = 0; i< contours_r.size(); i++ ){
      if (radius_r[i] > iMin_tracking_ball_size){Scalar color = Scalar( 0, 0, 255);
            drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );

            vector<float> ball_position_r;
            ball_position_r = pixel2point(center_r[i], radius_r[i]);
            float isx = ball_position_r[0];
            float isy = ball_position_r[1];
            float isz = ball_position_r[2];
            float radi = ball_position_r[3];
            string sx = floatToString(isx);
            string sy = floatToString(isy);
            string sz = floatToString(isz);
            ball_r_x.push_back(isx);
            ball_r_y.push_back(isy);
            ball_r_z.push_back(isz);
            ball_r_radius.push_back(radi);
		           count_r++;
               float x1, y1, x2, y2;
               x1, x2 =center_r[i].x,center_r[i+1].x;
               y1, y2 =center_r[i].y,center_r[i].y;
               float diff = pow((x1-x2),2)+pow((y1-y2),2);
               float l = radius_r[i]-radius_r[i+1];
            text = "Red ball:" + sx + "," + sy + "," + sz;
            putText(result, text, center_r[i],2,1,Scalar(0,255,0),2);
            circle( result, center_r[i], (int)radius_r[i], color, 2, 8, 0 );
            if (abs(l)<diff)i++;

        }
    }

    for( size_t i = 0; i< contours_b.size(); i++ ){
      if(radius_b[i] > iMin_tracking_ball_size){
            Scalar color = Scalar( 255, 0, 0);
            drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            vector<float> ball_position_b;
            ball_position_b = pixel2point(center_b[i], radius_b[i]);
            float isx = ball_position_b[0];
            float isy = ball_position_b[1];
            float isz = ball_position_b[2];
            float radi = ball_position_b[3];
            string sx = floatToString(isx);
            string sy = floatToString(isy);
            string sz = floatToString(isz);
            ball_b_x.push_back(isx);
            ball_b_y.push_back(isy);
            ball_b_z.push_back(isz);
            ball_b_radius.push_back(radi);
		          count_b++;
              float x1, y1, x2, y2;
              x1, x2 =center_b[i].x,center_b[i+1].x;
              y1, y2 =center_b[i].y,center_b[i].y;
              float diff = pow((x1-x2),2)+pow((y1-y2),2);
              float l = radius_b[i]-radius_b[i+1];
            text = "Blue ball:" + sx + "," + sy + "," + sz;
            putText(result, text, center_b[i],2,1,Scalar(0,255,0),2);
            circle( result, center_b[i], (int)radius_b[i], color, 2, 8, 0 );
            if (abs(l)<diff)i++;
        }
    }
cout<<count_b<< count_r<<endl;
msg.size_b = count_b;
msg.img_x_b = ball_b_x;
msg.img_y_b = ball_b_y;
msg.img_z_b = ball_b_z;
msg.size_r = count_r;
msg.img_x_r = ball_r_x;
msg.img_y_r = ball_r_y;
msg.img_z_r = ball_r_z;

if(count_b)
  { float i =center_b[0].y;
    cout<<ball_b_x[0]<<"\t"<<ball_b_z[0]<<endl;

}

    pub.publish(msg);

    // Show the frames: Here, the 6 final widnows or frames are displayed for the user to see.
    imshow("Video Capture",calibrated_frame);
    imshow("Object Detection_HSV_Red",hsv_frame_red);
    imshow("Object Detection_HSV_Blue",hsv_frame_blue);
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

vector<float> pixel2point(Point center, int radius){vector<float> position;
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
    Zc = sqrt(pow(Xc,2)+pow(Yc,2)+pow(Zc,2)-pow(0.25,2));  // 0x1
    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Zc);
    return position;
}

// Trackbar for image threshodling in HSV colorspace : Red : The functions had been declared and created, and now they are positioned in the relevant result frames. In this case, in the red ball's frames.
void on_low_h_thresh_trackbar_red(int, void *){low_h_r = min(high_h_r-1, low_h_r);
    setTrackbarPos("Low H","Object Detection_HSV_Red", low_h_r);
}
void on_high_h_thresh_trackbar_red(int, void *){high_h_r = max(high_h_r, low_h_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r);
}
void on_low_h2_thresh_trackbar_red(int, void *){low_h2_r = min(high_h2_r-1, low_h2_r);
    setTrackbarPos("Low H2","Object Detection_HSV_Red", low_h2_r);
}
void on_high_h2_thresh_trackbar_red(int, void *){high_h_r = max(high_h_r, low_h_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r);
}
void on_low_s_thresh_trackbar_red(int, void *){low_s_r = min(high_s_r-1, low_s_r);
    setTrackbarPos("Low S","Object Detection_HSV_Red", low_s_r);
}
void on_high_s_thresh_trackbar_red(int, void *){high_s_r = max(high_s_r, low_s_r+1);
    setTrackbarPos("High S", "Object Detection_HSV_Red", high_s_r);
}
void on_low_v_thresh_trackbar_red(int, void *){low_v_r= min(high_v_r-1, low_v_r);
    setTrackbarPos("Low V","Object Detection_HSV_Red", low_v_r);
}
void on_high_v_thresh_trackbar_red(int, void *){high_v_r = max(high_v_r, low_v_r+1);
    setTrackbarPos("High V", "Object Detection_HSV_Red", high_v_r);
}

// Trackbar for image threshodling in HSV colorspace : Blue : The functions had been declared and created, and now they are positioned in the relevant result frames. In this case, in the blue ball's frames.
void on_low_h_thresh_trackbar_blue(int, void *){low_h_b = min(high_h_b-1, low_h_b);
    setTrackbarPos("Low H","Object Detection_HSV_Blue", low_h_b);
}
void on_high_h_thresh_trackbar_blue(int, void *){high_h_b = max(high_h_b, low_h_b+1);
    setTrackbarPos("High H", "Object Detection_HSV_Blue", high_h_b);
}
void on_low_s_thresh_trackbar_blue(int, void *){low_s_b = min(high_s_b-1, low_s_b);
    setTrackbarPos("Low S","Object Detection_HSV_Blue", low_s_b);
}
void on_high_s_thresh_trackbar_blue(int, void *){high_s_b = max(high_s_b, low_s_b+1);
    setTrackbarPos("High S", "Object Detection_HSV_Blue", high_s_b);
}
void on_low_v_thresh_trackbar_blue(int, void *){low_v_b= min(high_v_b-1, low_v_b);
    setTrackbarPos("Low V","Object Detection_HSV_Blue", low_v_b);
}
void on_high_v_thresh_trackbar_blue(int, void *){high_v_b = max(high_v_b, low_v_b+1);
    setTrackbarPos("High V", "Object Detection_HSV_Blue", high_v_b);
}

// Trackbar for Canny edge algorithm : The trackbars for the Canny edge window are positioned here, for both the red and blue balls.
void on_canny_edge_trackbar_red(int, void *){setTrackbarPos("Min Threshold", "Canny Edge for Red Ball", lowThreshold_r);
}
void on_canny_edge_trackbar_blue(int, void *){setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball", lowThreshold_b);
}
