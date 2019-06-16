#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include "core_msgs/ball_position.h"
#include "core_msgs/roller_num.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include "opencv2/opencv.hpp"
#include "util_rtn.hpp"
#define POLICY LEFTMOST
#define WEBCAM
#define MYRIO

#define DISTANCE_TICKS 65

#define DIST(x1,y1,x2,y2) sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)))

#define ROTATE_CONST_SLOW 1.4f
#define TRANSLATE_CONST_SLOW 820.0f

#define DURATION 0.025f
#define COLLECT_THRESH_FRONT 0.1f
#define DISTANCE_TICKS_CL 75

#define TESTENV "demo-final"

#define RELIABLE
#define UNRELIABLE

/* State of our machine = SEARCH phase by default */
enum status machine_status = INIT;
enum status recent_status = INIT;
int target_blue_top(int policy);
int closest_blue_top();
/* Number of balls holding */
int ball_cnt = 0;

#ifdef WEBCAM
/* Blue balls */
int blue_cnt;
float blue_x[20];
float blue_y[20];
float blue_z[20];

float recent_target_b_x, recent_target_b_z;

/* Red balls */
int red_cnt;
float red_x[20];
float red_y[20];
float red_z[20];

/* Green balls */
int green_cnt;
float green_x[20];
float green_y[20];
float green_z[20];

/* CAM02 */
int blue_cnt_top;
int red_cnt_top;
int green_cnt_top;

float blue_x_top[20];
float blue_y_top[20];
float blue_z_top[20];
float red_x_top[20];
float red_y_top[20];
float red_z_top[20];
float green_x_top[20];
float green_y_top[20];
float green_z_top[20];

/* CAM03 */
size_t dupACKcnt = 0;
int return_mode = 0;
#endif

bool dirtybit_SG = false;

#ifdef MYRIO
int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];
#endif

#ifdef LIDAR
/* Absolute position relative to start pos */
float xpos_abs;
float ypos_abs;
float theta_abs;
#endif

float x_offset, y_offset, z_offset, x_offset_top, z_offset_top;
float downside_angle;

bool red_phase2 = false;
int timeout = 60;

float goal_theta;
float goal_x, goal_z;

void sigsegv_handler(int sig) {
  printf("(%s) program received SIGSEGV, Segmentation Fault. Ignoring\n", TESTENV);
  return;
}

bool use_myrio = true;

/* Main routine */
int main(int argc, char **argv)
{
    signal(SIGSEGV, sigsegv_handler); 	//인덱스 참조 잘못했을 때 프로그램 terminate 방지

    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    /* Argument parsing */
    int tag;
//offsets for control position offsets
//downside_angle is the angle from top camera
//set timeout to change phase to return when out of time.
    char x_offset_[5];
    char y_offset_[5];
    char z_offset_[5];
    char x_offset_top_[5];
    char z_offset_top_[5];
    char downside_angle_[6];
    char timeout_str[6];

    int flag = 0; //boolean variable to check argument content existence 
//initializing pointers to make all 0's.
    memset(x_offset_, 0, 6);
    memset(y_offset_, 0, 6);
    memset(z_offset_, 0, 6);
    memset(downside_angle_, 0, 6);


//Using argument parsing, we can set x, y, z offsets, use myrio and timeout in command.
//command example ) rosrun data_integration data_integrate_node -x -0.013 -z 0.18 -T 60 -m

    while((tag = getopt(argc, argv, "x:z:X:Z:mT:")) != -1) {
      switch(tag) {
        case 'x':
          flag |= 0x8; // 1 0 0 0
          memcpy(x_offset_, optarg, strlen(optarg));
          break;
        case 'z':
          flag |= 0x2; // 0 0 1 0 
          memcpy(z_offset_, optarg, strlen(optarg));
          break;
        case 'X':
          flag |= 0x10; // 1 0 0 0 0
          memcpy(x_offset_top_, optarg, strlen(optarg));
          break;
        case 'Z':
          flag |= 0x20;
          memcpy(z_offset_top_, optarg, strlen(optarg));
          break;
        case 'm':
          use_myrio = false;
          break;
        case 'T':
          flag |= 0x40;
          memcpy(timeout_str, optarg, strlen(optarg));
          break;
      }
    }

    x_offset = atof(x_offset_); //atof : alphabet to float
    z_offset = atof(z_offset_);
    x_offset_top = atof(x_offset_top_);
    z_offset_top = atof(z_offset_top_);
    timeout = atoi(timeout_str); //atoi : alphabet to int

//if flag not set, use default values as given:
// CAM01 : default x_ofs = -0.01m, z_ofs = 0.18m
    if(!(flag & 0x8)) x_offset = -0.01f; 
    if(!(flag & 0x2)) z_offset = 0.18f;
    if(!(flag & 0x10)) x_offset_top = 0;
    if(!(flag & 0x20)) z_offset_top = 0;
    if(!(flag & 0x40)) timeout = 60;

//debugging aiding message
    printf("(%s) start\n", TESTENV); //start message
    printf("(%s) camera offset : x=%.3f, y=%.3f, z=%.3f [m]\n", TESTENV, x_offset, y_offset, z_offset); //print offset values

//Subscribing to necessary ROS topics given WEBCAM defined
    #ifdef WEBCAM
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    ros::Subscriber sub2 = n.subscribe<core_msgs::ball_position_top>("/position_top", 1000, camera_Callback_top);
    ros::Subscriber sub3 = n.subscribe<core_msgs::roller_num>("/roller_num",1000, camera_Callback_counter);
    #endif
	
//If MYRIO is defined, then make socket to connect to Myrio and initialize it.
    #ifdef MYRIO
    if(use_myrio) {
    printf("(%s) Connecting to %s:%d\n", TESTENV, IPADDR, PORT);
    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

//앞서 초기화한 세팅으로 myrio와 connect하는데,
//If failed to connect, then print fail msg, close socket and return -1.
//If connected, print connet msg.
    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
      printf("(%s) Failed to connect\n", TESTENV);
      close(c_socket);
      return -1;
    }
    printf("(%s) Connected to %s:%d\n", TESTENV, IPADDR, PORT);
    }
    #endif
//finish connecting soket
	
	
    printf("(%s) Entering main routine...\n", TESTENV);
    printf("(%s) state = INIT\n", TESTENV);

    current_ticks = timer_ticks; //   1 timet_ticks = 1/40s (아래 main 함수 끝날 때 ros::Duration(DURATION).sleep();에서 value 정함.)

    while(ros::ok){
	//initializing 24byte array recieved from TCP IP (while loop 할 때마다 initialize해주지 않으면 전 loop의 data와 합쳐진다.)
      dataInit(); 
	//state transition을 확인하기 위해 state가 바뀔 때에만 프린트한다.
      if(recent_status != machine_status)
        std::cout << "(" << TESTENV << ") state = " << cond[(recent_status = machine_status)] << std::endl;
   	//timeout 시간을 넘기면 return phase로 status를 바꾼다.
	//timeout 설정한 이유: blue ball counter 문제에 대한 Backup Plan
	if(timer_ticks > 40 * timeout && !return_mode) {
        printf("(%s) switching to return mode after %.2f s of timeout\n", TESTENV, (float) timeout);
        machine_status = LIDAR_RETURN;
        return_mode = 1;
      }

/* switching machine_status based on pre-defined conditions */
      switch(machine_status) {
	/*INIT phase : start with go front 2m and move to SEARCH phase*/
        case INIT:
        {
          MSGE("go front 2m first") //printf function that print debugging msg per 1/4s (function defined in header "util_rtn")
          GO_FRONT
	// 2s (= 2m) 동안 GO_FRONT 후 SEARCH phase로 넘어간다.
          if(timer_ticks - current_ticks >= 80) 
            machine_status = SEARCH;
          break;
        }
	/*SEARCH phase :
	Searching blue ball. 
	If there is red ball, then move to RED_AVIODANCE phase.
	If there is any blue ball, align to target blue ball.
	If no ball is detected in both cameras, then search blue ball by clockwise rotation. */
        case SEARCH: 
        {
	// POLICY : target selecting policy. 1. leftmost, 2. centermost, 3. closest ball
	// default POLICY : leftmost
          int target_b = target_blue(POLICY); //leftmost_blue();
          int target_b_top = target_blue_top(POLICY); //leftmost_blue_top();
          if(target_b < 0) { //아래 카메라에서 blue ball이 보이지 않을 때
            if(target_b_top >= 0) { // 위 카메라에서 blue ball이 보일 때
	      //ball의 x 값이 +-0.3 범위에 없으면 TURN, 범위 안에 들어오면 GO_FRONT.
              float xpos_top_b = blue_x_top[target_b_top];
              if(xpos_top_b >= 0.3) TURN_RIGHT 
              else if(xpos_top_b <= -0.3) TURN_LEFT
              else GO_FRONT
            } else { 
	      // 아래, 위 카메라에서 둘 다 blue ball을 못찾으면 TURN_RIGHT해 blue ball을 찾는다.
              TURN_RIGHT
            }
          } else { //아래 카메라에서 blue ball이 보일 때 
	    //ball의 x 값이 +-0.15 범위에 없으면 TURN, 범위 안에 들어오면 APPROACH phase로 넘어간다.
            if(blue_x[target_b] > 0.15) {
              TURN_RIGHT // ROS_INFO("search - R");
            } else if(blue_x[target_b] < -0.15) {
              TURN_LEFT //  ROS_INFO("search - L");
            } else {
               machine_status = APPROACH;
            }
          }
	//target blue ball 보다 red ball이 가까이 있을 때 RED_AVOIDANCE phase로 넘어간다.
          if(red_in_range()) machine_status = RED_AVOIDANCE;
          break;
	}
	/*Approach phase :
	GO_FRONT toward target blue ball. Initial target is leftmost_blue ball in bottom camera.
	If blue ball is detected in bottom camera, then switch target to closest blue ball.
	( Target is always closest blue ball detected in bottom camera.)
	If target blue ball misaligned, then move to SEARCH phase, else move to COLLECT phase.*/
        case APPROACH: 
        {
          GO_FRONT //GO_FRONT toward target blue ball.
          int target_b = leftmost_blue();
          int target_b2 = closest_ball(BLUE);

          if(target_b2 != -1) target_b = target_b2; // target is closest blue ball detected in bottom camera.
	//If x postion of target is larger than 20cm, then move to SEARCH phase to align to target again.
          if(fabs(blue_x[target_b]) >= 0.20) { machine_status = SEARCH; } 
          else { 
	//If target is within 20cm from the vehicle, then move to COLLECT phase.
            if(blue_z[target_b] < 0.2) {
              machine_status = COLLECT;
            }
          }
	//If there is any red ball between target and vehicle, then move to RED_AVOIDANCE.
	// else, maintain previously determined status.
          if(red_in_range()) {
	    //(for error detection) If red in range but there's no red ball, then print assertion failed error.
            assert(closest_ball(RED) != -1); 
            if(red_z[closest_ball(RED)] <= blue_z[target_b]) {	    //If closest red ball is closer than target blue ball
              red_phase2 = false; 	//using red_phase2 for timer which is used in RED_AVOIDANCE phase.
              machine_status = RED_AVOIDANCE;
            }
          }

          break;
        }
	/* RED_AVOIDANCE phase : 
	Counter-clockwise rotation until red ball is out of range.(1)
	Then go front for 65 time_ticks ( 65* 1/40 s) (2)
	Lastly, move to search phase (during pickup mode) or search_green phase (during return mode) (3)*/
        case RED_AVOIDANCE:
        { //If red is not in sight and red_phase2 is false (set vehicle to go front)
          if(!red_in_range() && !red_phase2){ 
            red_phase2 = true; // change red_phase2 boolean to true to go front.
            printf("(%s) RED_AVOIDANCE_2\n", TESTENV);
            current_ticks = timer_ticks; //timer on
	  }
	  //If red_phase2 = true (turn left until the red ball is not in sight) (1)
          if(!red_phase2) TURN_LEFT
	  //If red_phase2 = True (go front for 65 * 1/40 s which is pre-defined based on both analytical and experimental test) (2)
          else {
		  GO_FRONT
	    // When timer becomes larger than distance_ticks (DISTANCE_TICKS: 일정거리 전진하는 변수 ( 65 * 1/40 s) )
            if(timer_ticks - current_ticks > DISTANCE_TICKS) { 
              red_phase2 = false;
	      //After go front, move phase to search_green (during return mode) or Search_blue (during pickup mode)
	      machine_status = (return_mode)? SEARCH_GREEN : SEARCH;
          }
        }
          break;
        }
	/* COLLECT phase :
	Turn on the roller and set target blue ball to closest ball in bottom camera.
	When target is too close, it can hide under the roller (collectable situation), then move to COLLECT2 phase to go front and pick up ball.
	Otherwise, target is still in sight then align to target, start timer and move to COLLECT2 phase.
	Then save x position and z position( distance) of recent target to use in COLLECT2.
	If there is any red ball between the target and vehicle at any case, then move to RED_AVOIDANCE phase.*/
        case COLLECT:
        {
          ROLLER_ON
	  // if there are more than one blue ball in the bottom camera sight, target the closest blue ball in bottom camera
          int target = closest_ball(BLUE);
          if(target==-1) target = leftmost_blue();
	  //If blue ball is too close, it can be out of sight (meaning collectable). Then move to COLLECT2 phase to collect hidden ball.
          if(target == -1){
            machine_status = COLLECT2; 
            printf("recent target blue was at (%.3f, %.3f)\n", recent_target_b_x, recent_target_b_z);
          }
 	  //If target is in sight, then align to target again, start timer and move to COLLECT2 phase.
          else { 
            float xpos = blue_x[target];
            float zpos = blue_z[target];
	    // align to target if x position of target is larger than +-1.3cm
            if(xpos > 0.013) {
              TURN_RIGHT_SLOW ROLLER_ON //printf("col-R\n");
            } else if(xpos < -0.013) {
              TURN_LEFT_SLOW ROLLER_ON //printf("col-L\n");
            } else {
	      // After aligning to target
              current_ticks = timer_ticks; // timer on
              machine_status = COLLECT2; // move to COLLECT2 phase
            }
          }
	  //If red ball is in sight
          if(red_in_range()) {
	    //If red ball is between the target and vehicle, move to RED_AVOIDANCE phase.
            if(red_z[closest_ball(RED)] <= blue_z[target]) machine_status = RED_AVOIDANCE;
          }
	  //Save x position and z position(distance) of target to use in COLLECT2 phase.
          if(target >= 0){
            recent_target_b_x = blue_x[target];
            recent_target_b_z = blue_z[target];
          }
          break;
        }
        /* COLLECT2 phase : 
	Go front for 40 * 1/ 40 s using timer and turn on the roller.
	After timer is more than 40 * 1/40 s, stop vehicle but keep roller on to collect ball completely.
	When timer becomes larger than DISTANCE_TICKS_CL ( 75 * 1/40 s) ,then move to SEARCH phase to collect next ball. */
	case COLLECT2:
        {
          if(timer_ticks - current_ticks <= 40) GO_FRONT //GO_FRONT until timer becomes 40.
	  ROLLER_ON //turn on the roller.
          if(timer_ticks - current_ticks > DISTANCE_TICKS_CL) { // If timer becomes larger than DISTANCE_TICKS_CL ( 75)
            machine_status = SEARCH; // move to SEARCH phase.
	    //If x position of target is smaller than 13.3cm, then count up.
	    //Use x position value to do ball count. Because when we used z position, ball count increased for both collected ball and disappeared ball in sight.
	    //As this does not always count balls correctly, We made ball-counter using another ball-counter camera.
            if(fabs(recent_target_b_x)<0.133)
              printf("(%s) collected ball. ball_count = %d\n",TESTENV , ++ball_cnt);
          }
          break;
        }
	/* LIDAR_RETURN phase : 
	- We made this code as back up plan when distant green ball is not detected with webcam.
	- But we did not use in demo as there was no problem in detecting green balls in the demo arena.
	If the ball count is equal to 3, then change to LIDAR_RETURN phase.
	If the lidar is not utilized (or not defined), then immediately change to SEARCH_GREEN phase.
	If the lidar is utilized, based on the absolute coordinates, move vehicle to the center ilne aligning with green balls.
	Then move to SEARCH_GREEN phase.*/
        case LIDAR_RETURN:
        {
	  // If the lidar is not utilized, move to SEARCH_GREEN phase.
          #ifndef LIDAR
          printf("(%s) LIDAR_RETURN : No LIDAR detected. Exitting.\n", TESTENV);
          machine_status = SEARCH_GREEN;
	  // If the lidar is utilized 
          #else 
	  //If the absolute angle of the vehicle is smaller than 170 degree or larger than 190 degree,
	  //adjust vehicle’s direction back to near 180 degree. 
	  if(theta_abs > 190.0f) {
            MSGE("LIDAR_RETURN : turn left")
            TURN_LEFT
          } else if(theta_abs < 170.0f) {
            MSGE("LIDAR_RETURN : turn right")
            TURN_RIGHT
          } else if(xpos_abs > 2.0f) { 
         //if the vehicle is away from the center line by 2 cm, adjust the lateral position of the vehicle towards the center line.
            MSGE("LIDAR_RETURN : go_front")
            GO_FRONT
          } else {
	 //After aligning, move to SEARCH_GREEN phase.
            machine_status = SEARCH_GREEN;
          }
	 //If red ball is detected in bottom camera, then move to RED_AVOIDANCE phase.
          if(red_in_range()){
            MSGE("LIDAR_RETURN : red ball detected")
            machine_status = RED_AVOIDANCE;
          }
          #endif
          break;
        }
        /* SEARCH_GREEN : 
	switch case breakdown based on number of green balls detected.
	Green ball Search & Alignment Case0: search, case1: approach, case 2: approach & align 
	case 0 : Vehicle rotates clockwise until detection.
	case 1 : If distance to the green ball is larger than 2m, then go front with aligning.
		After reaching 2m, slowly rotate clockwise to detect the other green ball.
	case 2 : Use feedback loop to align with basket.*/
	case SEARCH_GREEN:
        {
          switch(green_cnt_top) {
            case 0: //If no green ball is detected, rotate clockwise until green ball detection.
		{ TURN_RIGHT_MID break; }
            case 1: //If only 1 green ball is detected,
		{
	      //(for debugging) print spin msg with distance to green ball per 1/4 s.
	      if(!(timer_ticks%10)) printf("(%s) spin(green_cnt_top = %d), zpos = %.4f\n", TESTENV, green_cnt_top, green_z_top[0]);
              // If distance to the detected green ball is larger than 2m,
	      if(green_z_top[0] > 2.0f){
		 //calculate angle for feedback loop
		 float target_theta = RAD2DEG(atan(green_x_top[0]/green_z_top[0]));
		 // turn and go front to searching other green ball.
		if(target_theta < -20.0f) TURN_RIGHT
		else if(target_theta > 20.0f) TURN_LEFT
		else GO_FRONT
	      }
	      //If green ball is in range of 2m, slowly rotate clockwise to find other green ball.
 	      else { TURN_RIGHT_SLOW printf("turn_right : cnt=1, zpos = %.4f\n", green_z_top[0]); }
              break;
		}
            case 2: //If 2 green balls are detected,
            default: // back up plan when more than 3 green balls are detected.
            {	// using top camera to detect distant green balls
              float xg1 = green_x_top[0];
              float zg1 = green_z_top[0];
              float xg2 = green_x_top[1];
              float zg2 = green_z_top[1];
	      // Finding midpoint of green balls
              float mid_x = 0.5 * (xg1 + xg2); 
              float mid_z = 0.5 * (zg1 + zg2); 
	      //print msg of position of green balls every 1/4 s.
              if(!(timer_ticks % 10)) printf(" (%.3f, %.3f), (%.3f, %.3f) \n", xg1, zg1, xg2, zg2);
	      //calculate angle between midpoint of green balls and center line of camera.
              float theta = RAD2DEG(atan((zg2-zg1)/(xg2-xg1))); 
//If midpoint is larger than 2m, then make target_theta in range of +_10 degree and go front until green balls are in 2m.
if(mid_z > 2.0f) {
  float target_theta = RAD2DEG(atan(mid_x/mid_z));
  if(target_theta > 10.0f) TURN_RIGHT
  else if(target_theta < -10.0f) TURN_LEFT
  else GO_FRONT


} 
//When midpoint is within 2m, then align with degrees and x position.
else {

              if(theta < -10.0f){
		if(theta < -15.0f) TURN_RIGHT //If theta is smaller than -15 degree, turn right.
                else TURN_RIGHT_SLOW //If theta is in range of -10 to -15, turn right slowly for delicate alignment.
                MSGE("turn_right_slow")

              } else if(theta > 10.0f) { 
                if(theta > 15.0f) TURN_LEFT //If theta is larger than 15 degree, turn left.
		else TURN_LEFT_SLOW //If theta is in range of 10 to 15, turn left slowly for delicate alignment.
                MSGE("turn_left_slow")
 
              } else if(mid_x < -0.02f) { //If x position of midpoint is smaller than -2cm,
                if(mid_x < -0.10f) TRANSLATE_LEFT_3x //If x position of midpoint is smaller than -2cm, move left.
                else TRANSLATE_LEFT //If x position of midpoint is between -2cm and -10cm, move left slowly.
              } else if(mid_x > 0.02f) {  //If x position of midpoint is larger than +2cm,
                if(mid_x > 0.10f) TRANSLATE_RIGHT_3x //If x position of midpoint is larger than +10cm, move right.
                else TRANSLATE_RIGHT //If x position of midpoint is between +2cm and +10cm, move right slowly.
              } else {
		//After aligning to basket, go front.
                GO_FRONT
                MSGE("go_front")
              }

}
	      //If vehicle is perfectly aligned with the basket and 80cm away from the basket, move to RELEASE state.
              if(0.5*(zg1+zg2) <= 0.8){ 
		      printf("goal distance = %.4f\n", 0.5*(zg1+zg2)); 
		      goal_z = 0.5*(zg1+zg2); //save goal_z for using in RELEASE phase.
		      current_ticks = timer_ticks;
		      machine_status = RELEASE; 
	      } 
	    //If there is any red ball in sight, then move to RED_AVOIDANCE phase.
            if(red_in_range())
              machine_status = RED_AVOIDANCE;
            break;
            }
          }
          break;
        }
	      
/* RELEASE phase :
If lidar is not utilzed, vehicle goes front based on the distance to basket calculated in SEARCH_GREEN phase.
Then it rotates the roller in reverse direction to release the balls into the basket.*/
        case RELEASE:
        {
	//If lidar is not utilzed,           
	  #ifndef LIDAR
	//goes front based on distance to basket calculates with goal_z which is from SEARCH_GREEN phase.
	// goal_front_ticks (= 90 * goal_z(m)) is the time to go front for goal_z (m) is determined both analytically and experimentally.
          uint32_t goal_front_ticks = (uint32_t) (90.0f * (goal_z));
	//1st step : go front
          if(timer_ticks-current_ticks < goal_front_ticks) {
            MSGE("RELEASE - go front")
            GO_FRONT
	//2nd step : release balls, (we added 100 timeticks to goal_front_ticks for correctly get into basket.)
          } else if(timer_ticks - current_ticks < 100 + goal_front_ticks) {
            MSGE("RELEASE - roller_reverse")
            ROLLER_REVERSE // reversly roller on to release balls.
          } else { 
	//After releasing balls, print total time and terminate.
	    printf("(%s) elapsed time = %.4f sec\n", TESTENV, 0.025 * timer_ticks);
            PANIC("RELEASE_TERMINATE : should have released 3 balls.") 
          }
	//If lidar is utilzed,
          #else
          if(xpos_abs > 0.02f) {
            MSGE("RELEASE - go front(feedback)")
            GO_FRONT
            current_ticks = 0;
          } else {
            MSGE("RELEASE - roller_reverse")
            ROLLER_REVERSE
            current_ticks++;
          }

          if(current_ticks > 500){
	    printf("(%s) elapsed time = %.4f sec\n", TESTENV, 0.025 * timer_ticks);
            PANIC("RELEASE : terminating. should have released 3 balls.")
	  }
          #endif

          break;
        }
//When phase error, print assertion error.
        default:
          assert(0);

      }


      /* Send control data */
     /*myRIO에 array 보내는 코드*/
	    
      #ifdef MYRIO
      if(use_myrio) {
      size_t written = write(c_socket, data, sizeof(data)); //Send 24 bit array named 'data' to TCP/IP by write function.
	  
      if(DEBUG) printf("%d bytes written\n", (int) written); //print msgwhether TCP/IP is working well.
      }
      #endif

	    ros::Duration(DURATION).sleep(); //make timer_ticks to increase regularly (duration is predefined to 1/40s)
	    ros::spinOnce();
      timer_ticks++; //increase time_ticks 1.
    }

    close(c_socket); //close socket
    ros::shutdown();

    return -1;
} //end of main function.

#ifdef WEBCAM //If using webcam,
/* camera_Callback : Updates position/ball_count of all colors */
//bottom camera callback function
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  /* Step 1. Fetch data from message */
  /*blue*/
  //get the number of blue balls
  int b_cnt = position->size_b;
  blue_cnt = position->size_b;

  for(int i=0; i<b_cnt; i++) {
    // transform the matrix
    float x_pos = position->img_x_b[i];
    float y_pos = position->img_y_b[i];
    float z_pos = position->img_z_b[i];

    /* TODO : transform (Camera coordinate)->(LLF) */
    //As the position of webcam and roller id different, we find proper offset to change coordinate to roller.
    blue_x[i] = x_pos - x_offset;
    blue_z[i] = z_pos - z_offset;
  }
  //get the number of red balls
  /*red*/
  int r_cnt = position->size_r;
  red_cnt = position->size_r;

  for(int i=0; i<r_cnt; i++) {
    // transform the matrix
    float x_pos = position->img_x_r[i];
    float y_pos = position->img_y_r[i];
   float z_pos = position->img_z_r[i];

    /* TODO : transform (Camera coordinate)->(LLF) */
    //As the position of webcam and roller id different, we find proper offset to change coordinate to roller.
    red_x[i] = x_pos - x_offset;

	red_z[i] = z_pos - z_offset;
	  
  }
  /*green*/
  //get the number of green balls
  int g_cnt = position->size_g;
  green_cnt = position->size_g;

  for(int i=0; i<g_cnt; i++) {
      // transform the matrix
    float x_pos = position->img_x_g[i];
    float y_pos = position->img_y_g[i];
    float z_pos = position->img_z_g[i];
    //As the position of webcam and roller id different, we find proper offset to change coordinate to roller.
    green_x[i] = x_pos - x_offset;
    green_z[i] = z_pos - z_offset;
  }

}

/* callback 2 */
//top camera call back function (same algorithm with bottom camera callback.)
void camera_Callback_top(const core_msgs::ball_position_top::ConstPtr& position)
{
  /* Step 1. Fetch data from message */
  /*blue*/
  int b_cnt_top = position->size_b;
  blue_cnt_top = position->size_b;

  for(int i=0; i<b_cnt_top; i++) {
    // transform the matrix
    float x_pos = position->img_x_b[i];
    float y_pos = position->img_y_b[i];
    float z_pos = position->img_z_b[i]; 

    /* TODO : transform (Camera coordinate)->(LLF) */
    blue_x_top[i] = x_pos - x_offset_top;
    blue_z_top[i] = z_pos - z_offset_top;
  }
	
  /*red*/
  int r_cnt_top = position->size_r;
  red_cnt_top = position->size_r;

  for(int i=0; i<r_cnt_top; i++) {
    float x_pos = position->img_x_r[i];
    float y_pos = position->img_y_r[i];
    float z_pos = position->img_z_r[i];

    /* TODO : transform (Camera coordinate)->(LLF) */
    red_x_top[i] = x_pos - x_offset;
    red_z_top[i] = z_pos - z_offset;
  }

  /* Green */
  int g_cnt_top = position->size_g;
  green_cnt_top = position->size_g;

  for(int i=0; i<g_cnt_top; i++) {
    float x_pos = position->img_x_g[i];
    float y_pos = position->img_y_g[i];
    float z_pos = position->img_z_g[i];

    /* TODO : transform (Camera coordinate)->(LLF) */
    green_x_top[i] = x_pos - x_offset;
    green_z_top[i] = z_pos - z_offset;
  }
}
//ball counter camera call back function
void camera_Callback_counter(const core_msgs::roller_num::ConstPtr& cnt)
{
  //get binary value whether the number of collected blue balls are 3 or less.
  int shift = cnt->size_b;
  //If there are 3 ball and not return mode,
  if(shift && !return_mode) {
    printf("(%s) dupACKcnt = %d\n", TESTENV, (int) ++dupACKcnt); //increase dupACKcnt variable 1
    if(dupACKcnt >= 3) {  // If dupACKcnt is larger than 3,
      if(machine_status == SEARCH || !return_mode) { //If phase is not in SEARCH and not return_mode,
        machine_status = LIDAR_RETURN; //move to LIDAR_RETURN phase to release balls.
        return_mode = 1; // turn on return_mode
      }
    }
  }
  else // If ball counter is 0, then init dupACKcnt value.
    dupACKcnt = 0;

}

//detect ball with colors.
bool ball_in_range(enum color ball_color) {
//If color of ball to detect is blue,
  if(ball_color == BLUE) {
    for(int i=0; i<blue_cnt; i++) {
	//If blue ball is inside  the range of +/- 0.034 m lateral  and 0.3 m longitudinal, return true
      if(fabs(blue_x[i])<=0.034 && blue_z[i] <= 0.3)
        return true;
    }
    return false; //else no blue ball is in range, return false.

  } else if(ball_color == RED) { //If color of ball to detect is red,
    return red_in_range(); // go to red_in_range

  } else {
    return false; //no red ball is detected then return false.
  }
}
//If red ball is inside the range of +/- 0.25 m and 0.3m longitudinal, return true, else false.
bool red_in_range() {
  for(int i=0; i<red_cnt; i++) {
    if(fabs(red_x[i])<0.25 && red_z[i] <= 0.3)
      return true;
  }
  return false;
}

/*
 * leftmost green()
 * return leftmost blue visible in sight in bottom camera
 * return -1 if invisible
 */
int leftmost_green() {
  if(!green_cnt)
    return -1;

  float xpos = green_x[0];
  int min_idx = 0;
// find ball index which has least x posiiton
  for(int i=0; i<green_cnt; i++) {
    if(green_x[i] <= xpos) {
      xpos = green_x[i];
      min_idx = i;
    }
  }
  return min_idx;
}
/*
 * rightmost green()
 * return rightmost blue visible in sight in bottom camera
 * return -1 if invisible
 */
int rightmost_green() {
  if(!green_cnt)
    return -1;

  float xpos = green_x[0];
  int max_idx = 0;

  for(int i=0; i<green_cnt; i++) {
    if(green_x[i] >= xpos)
      xpos = green_x[(max_idx = i)];
  }

  return max_idx;
}
/*
 * leftmost green_top()
 * return leftmost blue visible in sight in top camera
 * return -1 if invisible
 */
int leftmost_green_top() {
  if(!green_cnt_top)
    return -1;

  float xpos = green_x_top[0];
  int min_idx = 0;

  for(int i=0; i<green_cnt_top; i++) {
    if(green_x_top[i] <= xpos) {
      xpos = green_x_top[i];
      min_idx = i;
    }
  }
  return min_idx;
}


/*
 * leftmost blue()
 * return leftmost blue visible in sight in bottom camera
 * return -1 if invisible
 */
int leftmost_blue() {
  if(!blue_cnt)
    return -1;

  float xpos = blue_x[0];
  int min_idx = 0;

  for(int i=0; i<blue_cnt; i++) {
    if(blue_x[i] <= xpos) {
      xpos = blue_x[i];

      min_idx = i;
    }
  }
  return min_idx;
}
/*
 * leftmost blue()
 * return leftmost blue visible in sight with top camera
 * return -1 if invisible
 */
int leftmost_blue_top() {
  if(!blue_cnt_top)
    return -1;

  float xpos = blue_x_top[0];
  int min_idx = 0;

  for(int i=0; i<blue_cnt_top; i++) {
    if(blue_x_top[i] <= xpos) {
      xpos = blue_x_top[i];

      min_idx = i;
    }
  }
  return min_idx;
}

/*
 * target_blue(int policy)
 * policy : LEFTMOST(0), CENTERMOST(1), CLOSEST(2)
 * go to each function
 */
int target_blue(int policy) {
  switch(policy) {
    case LEFTMOST:
      return leftmost_blue();
    case CENTERMOST:
      return centermost_blue();
    case CLOSEST:
      return closest_ball(BLUE);
    default: // for error that undefined policy
      PANIC("Undefined ball policy");
  }
}

/*
 * target_blue(int policy)
 * policy : LEFTMOST(0), CENTERMOST(1), CLOSEST(2)
 * go to each function.
 */
int target_blue_top(int policy) {
  switch(policy) {
    case LEFTMOST:
      return leftmost_blue_top();
    case CENTERMOST:
      return -1;
    case CLOSEST:
      return closest_blue_top();
    default: // for error that undefined policy
      PANIC("Undefined ball policy");
  }
}


/*
 * centermost blue()
 * return index of centermost blue ball amongst visible ones in bottom camera
 * -1 if no blue ball is in sight
 */
int centermost_blue() {
  if(!blue_cnt) //If no ball is detected then return -1
    return -1;

  float xpos_abs = fabs(blue_x[0]);
  int min_idx = 0;
//find blue ball which has minimum absolute x value.
  for(int i=0; i<blue_cnt; i++) {
    if(fabs(blue_x[i]) < xpos_abs) {
      xpos_abs = fabs(blue_x[i]);
      min_idx = i;
    }
  }
  return min_idx;
}

/*
 * closest_blue_top()
 * return index of closest blue ball amongst visible ones in top camera
 * -1 if no blue ball is in sight
 */
int closest_blue_top() {
      int result_idx = -1;

      if(!blue_cnt_top)
        return -1;

      float front_dist = blue_z[0];

      for(int i=0; i<blue_cnt_top; i++){
        if(blue_z_top[i] <= front_dist){
          front_dist = blue_z_top[i];
          result_idx = i;
        }
      }

      return result_idx;
}

/*
 * closest_ball()
 * return index of closest ball which has color defined in function amongst visible ones in bottom camera
 * -1 if no ball with that color is in sight
 */
int closest_ball(enum color ball_color) {
  switch(ball_color) {
    case BLUE: // If blue ball
    {
      int result_idx = -1;

      if(!blue_cnt) //no blue ball is detected then return -1
        return -1;

      float front_dist = blue_z[0];
	//find index of closest blue ball
      for(int i=0; i<blue_cnt; i++){
        if(blue_z[i] <= front_dist){
          front_dist = blue_z[i];
          result_idx = i;
        }
      }
	//return index
      return result_idx;
    }
    case RED:
    {
      int result_idx = -1;

      if(!red_cnt) //no red ball is detected then return -1
        return -1;
	//find index of closest red ball
      float front_dist = red_z[0];
      for(int i=0; i<red_cnt; i++){
        if(red_z[i] <= front_dist){
          front_dist = red_z[i];
          result_idx = i;
        }
      }
	 //return index
        return result_idx;
      break;
    }
    case GREEN:
    {
      int result_idx = -1;

      if(!green_cnt)//no green ball is detected then return -1
        return -1;
	//find index of closest green ball
      float front_dist = green_z[0];
      for(int i=0; i<green_cnt; i++) {
        if(green_z[i] <= front_dist) {
          front_dist = green_z[i];
          result_idx = i;
        }
      }
	//return index
      return result_idx;
    }
    default: //for error that color variable is not in blue, red, green.
      { return -1; }
  }
}

/*
 * furthest_green()
 * Traverse through green_z
 * returns INDEX of maximum dist Z
 * This is redundant, but is implemented as senitel
 */
int furthest_green() {
  int result_idx = 0;

  float front_dist = green_z[0];
  for(int i=0; i<green_cnt; i++) {
    if(green_z[i] >= front_dist) {
      front_dist = green_z[i];
      result_idx = i;
    }
  }
  return result_idx;
}
#endif
//finished webcam

/* lidar*/
#ifdef LIDAR
void lidar_Callback(const lidar::coor::ConstPtr& pos)
{
  /* Update absolute position using pos */
  xpos_abs = pos->coor_x;
  ypos_abs = pos->coor_y;
  theta_abs = pos->coor_theta;

  if(DEBUG) printf("(%s) lidar callback! %.3f %.3f %.3f \n", TESTENV, pos->coor_x, pos->coor_y, pos->coor_theta);


}
#endif
//finished lidar

void dataInit()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
	data[5] = 0; //ry*data[7];
	data[6] = 0; //GamepadStickAngle(_dev, STICK_RIGHT);
	data[7] = 0; //GamepadStickLength(_dev, STICK_RIGHT);
	data[8] = 0; //GamepadTriggerLength(_dev, TRIGGER_LEFT);
	data[9] = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT);
	data[10] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP);
	data[11] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
	data[12] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
	data[13] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
	data[14] = 0; //GamepadButtonDown(_dev, BUTTON_A); // duct on/off
	data[15] = 0; //GamepadButtonDown(_dev, BUTTON_B);
	data[16] = 0; //GamepadButtonDown(_dev, BUTTON_X);
	data[17] = 0; //GamepadButtonDown(_dev, BUTTON_Y);
	data[18] = 0; //GamepadButtonDown(_dev, BUTTON_BACK);
	data[19] = 0; //GamepadButtonDown(_dev, BUTTON_START);
	data[20] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
	data[21] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
	data[22] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
	data[23] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);
}
