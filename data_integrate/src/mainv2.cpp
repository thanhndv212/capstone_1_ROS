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
int SG_sema = 0;

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

// CAM01 : default x_ofs = -0.013, z_ofs = 0.130

/* Main routine */
int main(int argc, char **argv)
{
    signal(SIGSEGV, sigsegv_handler);

    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    /* Argument parsing */
    int tag;

    char x_offset_[5];
    char y_offset_[5];
    char z_offset_[5];
    char x_offset_top_[5];
    char z_offset_top_[5];
    char downside_angle_[6];
    char timeout_str[6];

    int flag = 0;

    memset(x_offset_, 0, 6);
    memset(y_offset_, 0, 6);
    memset(z_offset_, 0, 6);
    memset(downside_angle_, 0, 6);

    while((tag = getopt(argc, argv, "x:z:X:Z:mT:")) != -1) {
      switch(tag) {
        case 'x':
          flag |= 0x8;
          memcpy(x_offset_, optarg, strlen(optarg));
          break;
        case 'z':
          flag |= 0x2;
          memcpy(z_offset_, optarg, strlen(optarg));
          break;
        case 'X':
          flag |= 0x10;
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

    x_offset = atof(x_offset_);
    z_offset = atof(z_offset_);
    x_offset_top = atof(x_offset_top_);
    z_offset_top = atof(z_offset_top_);
    timeout = atoi(timeout_str);

    if(!(flag & 0x8)) x_offset = -0.01f;
    if(!(flag & 0x2)) z_offset = 0.18f;
    if(!(flag & 0x10)) x_offset_top = 0;
    if(!(flag & 0x20)) z_offset_top = 0;
    if(!(flag & 0x40)) timeout = 60;


    printf("(%s) start\n", TESTENV);
    printf("(%s) camera offset : x=%.3f, y=%.3f, z=%.3f [m]\n", TESTENV, x_offset, y_offset, z_offset);

    #ifdef LIDAR
    ros::Subscriber sub = n.subscribe<lidar::coor>("/lidar_coor", 1000, lidar_Callback);
    #endif

    #ifdef WEBCAM
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    ros::Subscriber sub2 = n.subscribe<core_msgs::ball_position_top>("/position_top", 1000, camera_Callback_top);
    ros::Subscriber sub3 = n.subscribe<core_msgs::roller_num>("/roller_num",1000, camera_Callback_counter);
    #endif

		dataInit();

    #ifdef MYRIO
    if(use_myrio) {
    printf("(%s) Connecting to %s:%d\n", TESTENV, IPADDR, PORT);
    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
      printf("(%s) Failed to connect\n", TESTENV);
      close(c_socket);
      return -1;
    }
    printf("(%s) Connected to %s:%d\n", TESTENV, IPADDR, PORT);
    }
    #endif

    printf("(%s) Entering main routine...\n", TESTENV);
    printf("(%s) state = INIT\n", TESTENV);

    current_ticks = timer_ticks;

    while(ros::ok){
      dataInit();

      if(recent_status != machine_status)
        std::cout << "(" << TESTENV << ") state = " << cond[(recent_status = machine_status)] << std::endl;

      if(timer_ticks > 40 * timeout && !return_mode) {
        printf("(%s) switching to return mode after %.2f s of timeout\n", TESTENV, (float) timeout);
        machine_status = LIDAR_RETURN;
        return_mode = 1;
      }


      /* switching between states */
      switch(machine_status) {
        case INIT:
        {
          MSGE("go front 2m first")
          GO_FRONT

          /* Initiate SEARCH */
          if(timer_ticks - current_ticks >= 80)
            machine_status = SEARCH;

          break;
        }
        case SEARCH:
        {
          int target_b = target_blue(POLICY); //leftmost_blue();
          int target_b_top = target_blue_top(POLICY); //leftmost_blue_top();

          if(target_b < 0) {
            if(target_b_top >= 0) {
              float xpos_top_b = blue_x_top[target_b_top];
              if(xpos_top_b >= 0.3) TURN_RIGHT
              else if(xpos_top_b <= -0.3) TURN_LEFT
              else GO_FRONT
            } else {
              TURN_RIGHT
            }
          } else {
            if(blue_x[target_b] > 0.15) {
              TURN_RIGHT // ROS_INFO("search - R");
            } else if(blue_x[target_b] < -0.15) {
              TURN_LEFT //  ROS_INFO("search - L");
            } else {
               machine_status = APPROACH;
            }
          }

          if(red_in_range()) machine_status = RED_AVOIDANCE;
          break;
        }
        case APPROACH:
        {
          GO_FRONT
          int target_b = leftmost_blue();
          int target_b2 = closest_ball(BLUE);

          if(target_b2 != -1) target_b = target_b2;

          if(fabs(blue_x[target_b]) >= 0.20) { machine_status = SEARCH; }
          else {
            if(blue_z[target_b] < 0.2) {
              machine_status = COLLECT;
            }
          }

          if(red_in_range()) {
            assert(closest_ball(RED) != -1);
            if(red_z[closest_ball(RED)] <= blue_z[target_b]) {
              red_phase2 = false;
              machine_status = RED_AVOIDANCE;
            }
          }

          break;
        }
        case RED_AVOIDANCE:
        {
          if(!red_in_range() && !red_phase2){
            red_phase2 = true;
            printf("(%s) RED_AVOIDANCE_2\n", TESTENV);
            current_ticks = timer_ticks;
          }

          if(!red_phase2) TURN_LEFT
          else {
            GO_FRONT

            if(timer_ticks - current_ticks > DISTANCE_TICKS) {
              red_phase2 = false;
              machine_status = (return_mode)? SEARCH_GREEN : SEARCH;
          }
        }
          break;
        }
        case COLLECT:
        {
          ROLLER_ON

          int target = closest_ball(BLUE);
          if(target==-1) target = leftmost_blue();

          if(target == -1){
            machine_status = COLLECT2;
            printf("recent target blue was at (%.3f, %.3f)\n", recent_target_b_x, recent_target_b_z);
          }
          else {
            float xpos = blue_x[target];
            float zpos = blue_z[target];

            if(xpos > 0.013) {
              TURN_RIGHT_SLOW ROLLER_ON //printf("col-R\n");
            } else if(xpos < -0.013) {
              TURN_LEFT_SLOW ROLLER_ON //printf("col-L\n");
            } else {
              current_ticks = timer_ticks;
              machine_status = COLLECT2;
            }
          }

          if(red_in_range()) {
            if(red_z[closest_ball(RED)] <= blue_z[target]) machine_status = RED_AVOIDANCE;
          }
          if(target >= 0){
            recent_target_b_x = blue_x[target];
            recent_target_b_z = blue_z[target];
          }
          break;
        }
        case COLLECT2:
        {
          if(timer_ticks - current_ticks <= 40) GO_FRONT
	  ROLLER_ON
          if(timer_ticks - current_ticks > DISTANCE_TICKS_CL) {
            machine_status = SEARCH;

            if(fabs(recent_target_b_x)<0.133)
              printf("(%s) collected ball. ball_count = %d\n",TESTENV , ++ball_cnt);
          }
          break;
        }
        case LIDAR_RETURN:
        {
          #ifndef LIDAR
          printf("(%s) LIDAR_RETURN : No LIDAR detected. Exitting.\n", TESTENV);
          machine_status = SEARCH_GREEN;

          #else
          if(theta_abs > 190.0f) {
            MSGE("LIDAR_RETURN : turn left")
            TURN_LEFT
          } else if(theta_abs < 170.0f) {
            MSGE("LIDAR_RETURN : turn right")
            TURN_RIGHT
          } else if(xpos_abs > 2.0f) {
            MSGE("LIDAR_RETURN : go_front")
            GO_FRONT
          } else {
            machine_status = SEARCH_GREEN;
          }

          if(red_in_range()){
            MSGE("LIDAR_RETURN : red ball detected")
            machine_status = RED_AVOIDANCE;
          }
          #endif
          break;
        }
        case SEARCH_GREEN:
        {
          switch(green_cnt_top) {
            case 0:
		{ TURN_RIGHT_MID break; }
            case 1:
		{
	      if(!(timer_ticks%10)) printf("(%s) spin(green_cnt_top = %d), zpos = %.4f\n", TESTENV, green_cnt_top, green_z_top[0]);
              if(green_z_top[0] > 2.0f){
		 float target_theta = RAD2DEG(atan(green_x_top[0]/green_z_top[0]));
		if(target_theta < -20.0f) TURN_RIGHT
		else if(target_theta > 20.0f) TURN_LEFT
		else GO_FRONT
	      }
 	      else { TURN_RIGHT_SLOW printf("turn_right : cnt=1, zpos = %.4f\n", green_z_top[0]); }
              break;
		}
            case 2:
            default:
            {
              float xg1 = green_x_top[0];
              float zg1 = green_z_top[0];
              float xg2 = green_x_top[1];
              float zg2 = green_z_top[1];

              float mid_x = 0.5 * (xg1 + xg2);
              float mid_z = 0.5 * (zg1 + zg2);

              if(!(timer_ticks % 10)) printf(" (%.3f, %.3f), (%.3f, %.3f) \n", xg1, zg1, xg2, zg2);

              float theta = RAD2DEG(atan((zg2-zg1)/(xg2-xg1)));
if(mid_z > 2.0f) {
  float target_theta = RAD2DEG(atan(mid_x/mid_z));
  if(target_theta > 10.0f) TURN_RIGHT
  else if(target_theta < -10.0f) TURN_LEFT
  else GO_FRONT


} else {

              if(theta < -10.0f){
		if(theta < -15.0f) TURN_RIGHT
                else TURN_RIGHT_SLOW
                MSGE("turn_right_slow")
                if(SG_sema) dirtybit_SG = true;
              } else if(theta > 10.0f) {
                if(theta > 15.0f) TURN_LEFT
		else TURN_LEFT_SLOW
                MSGE("turn_left_slow")
                if(SG_sema) dirtybit_SG = true;
              } else if(mid_x < -0.02f) {
                SG_sema = 1;
                if(mid_x < -0.10f) TRANSLATE_LEFT_3x
                else TRANSLATE_LEFT //_SLOW
              } else if(mid_x > 0.02f) {
                SG_sema = 1;
                if(mid_x > 0.10f) TRANSLATE_RIGHT_3x
                else TRANSLATE_RIGHT //_SLOW
              } else {
                GO_FRONT
                MSGE("go_front")
              }
}
              if(0.5*(zg1+zg2) <= 0.8){ printf("goal distance = %.4f\n", 0.5*(zg1+zg2)); goal_z = 0.5*(zg1+zg2); current_ticks = timer_ticks; machine_status = RELEASE; } //machine_status = APPROACH_GREEN;

            if(red_in_range())
              machine_status = RED_AVOIDANCE;

            break;
            }
          }



#ifdef UNUSED
          int target_g = leftmost_green();
          int target_g_top = leftmost_green_top();
          if(target_g_top >= 0) {
            float xpos = green_x_top[target_g_top];
            float zpos = green_z_top[target_g_top];

            if(xpos > 0.2){
              TURN_RIGHT
              if(!(timer_ticks%10)) printf("(%s) SEARCH_GREEN : turn right\n", TESTENV);
            } else if(xpos < -0.2) {
              TURN_LEFT
              if(!(timer_ticks%10)) printf("(%s) SEARCH_GREEN : turn_left\n", TESTENV);
            } else {
              GO_FRONT
              if(!(timer_ticks%10)) printf("(%s) SEARCH_GREEN : go_front\n", TESTENV);
            }

            if(zpos <= 0.8) {
              machine_status = APPROACH_GREEN;
            }

          } else TURN_RIGHT
#endif
          break;
        }
        case APPROACH_GREEN:
        {
          switch(green_cnt) {
            case 0:
            {
              TURN_RIGHT
              if(!(timer_ticks % 10)) printf("(%s) APPROACH_GREEN : green_cnt = 0, turn right\n", TESTENV);
              break;
            }
            case 1:
            {
              float xg1 = green_x[0];
              float zg1 = green_z[0];

              if(xg1 > 0 || 1) {
                TURN_RIGHT_SLOW
                if(!(timer_ticks%10)) printf("(%s) APPROACH_GREEN : green_cnt = 1, turn_right\n", TESTENV);
              } else if(xg1 < 0 && 0) {
                TURN_LEFT_SLOW
                if(!(timer_ticks%10)) printf("(%s) APPROACH_GREEN : green_cnt = 1, turn_left\n", TESTENV);
              }

              break;
            }
            case 2:
            {
              if(!(timer_ticks%10)) printf("(%s) APPROACH_GREEN : green_cnt = 2\n", TESTENV);

              float xg1 = green_x[0];
              float xg2 = green_x[1];
              float zg1 = green_z[0];
              float zg2 = green_z[1];

              /* Make sure one ball is recognized as 2 */
              if(DIST(xg1,zg1,xg2,zg2) <= 0.05) { TURN_RIGHT break; }

              float degree = atan((zg2-zg1)/(xg2-xg1));

              float mid_x = 0.5 * (xg1 + xg2);
              float mid_z = 0.5 * (zg1 + zg2);

              float mid_x_trn = mid_x * cos(degree) + mid_z * sin(degree);
              float mid_z_trn = mid_z * cos(degree) - mid_x * sin(degree);

              if(mid_x >= 0.1f) {
                if(!(timer_ticks%10)) printf("(%s) midpoint_x = %.3f : turn_right\n", TESTENV, mid_x);
                TURN_RIGHT_SLOW
              } else if(mid_x <= -0.1f) {
                TURN_LEFT_SLOW
                if(!(timer_ticks%10)) printf("(%s) midpoint_x = %.3f : turn_left\n", TESTENV, mid_x);
              } else {
                if(abs((green_x[0])<= 0.25 && abs((green_x[1])<=0.25) )) {
                  float xg1_ = green_x[0];
                  float xg2_ = green_x[1];
                  float zg1_ = green_z[0];
                  float zg2_ = green_z[1];

                  float degree_ = atan((zg2-zg1)/(xg2-xg1));
                  float mid_x_ = 0.5 * (xg1 + xg2);
                  float mid_z_ = 0.5 * (zg1 + zg2);

                  float mid_x_trn_ = mid_x * cos(degree_) + mid_z * sin(degree_);
                  float mid_z_trn_ = mid_z * cos(degree_) - mid_x * sin(degree_);

                  goal_theta = RAD2DEG(degree_);
                  goal_x = mid_x_trn_;
                  goal_z = mid_z_trn_;
                  printf("(%s) angular offset = %.3f deg, x_ofs = %.3f, z_ofs = %.3f\n", TESTENV, RAD2DEG(degree_), mid_x_trn_, mid_z_trn_);

                  machine_status = APPROACH_GREEN_2;
                  current_ticks = timer_ticks;

                }
              }

              int target = closest_ball(GREEN);
              float t_z = green_z[target];

             break;
            }
            default:
            {
              /* Evaluation over UNRELIABLE openCV */
              printf("(%s) APPROACH_GREEN : evaluation in unreliable mode : there are %d green balls\n",TESTENV, green_cnt);
              float xg1 = green_x[leftmost_green()];
              float xg2 = green_x[rightmost_green()];
              float zg1 = green_z[leftmost_green()];
              float zg2 = green_z[rightmost_green()];

              float mid_x = 0.5 * (xg1 + xg2);
              float mid_z = 0.5 * (zg1 + zg2);

              float degree_ = atan((zg2-zg1)/(xg2-xg1));

              float mid_x_trn_ = mid_x * cos(degree_) + mid_z * sin(degree_);
              float mid_z_trn_ = mid_z * cos(degree_) - mid_x * sin(degree_);

              goal_theta = RAD2DEG(degree_);
              goal_x = mid_x_trn_;
              goal_z = mid_z_trn_;

              printf("(%s) angular offset = %.3f deg, x_ofs = %.3f, z_ofs = %.3f\n", TESTENV, RAD2DEG(degree_), mid_x_trn_, mid_z_trn_);

              machine_status = APPROACH_GREEN_2;
              current_ticks = timer_ticks;

              break;
            }

          }

          break;
        }

        /*
         * APPROACH_GREEN_2
         * open-loop position control based on position evaluation of APPROACH_GREEN_1
         * 1) ROTATE theta-ofs, 2) TRANSLATE X-ofs, 3) phase shift to APPROACH_GREEN_3
         */
        case APPROACH_GREEN_2:
        {
          uint32_t goal_rotate_ticks = (uint32_t) (ROTATE_CONST_SLOW * fabs(goal_theta));
          uint32_t goal_translate_ticks = (uint32_t) (TRANSLATE_CONST_SLOW * fabs(goal_x));

          if(!(timer_ticks %10)) printf("(%s) estimated rotate ticks = %d, translate ticks = %d\n",TESTENV,(int) goal_rotate_ticks, (int) goal_translate_ticks);

          if(timer_ticks - current_ticks < goal_rotate_ticks) {
            MSGE("openloop - rotation")
            if(goal_theta > 0) TURN_LEFT_SLOW
            else if(goal_theta < 0) TURN_RIGHT_SLOW
          } else if((timer_ticks - current_ticks >= goal_rotate_ticks) && (timer_ticks - current_ticks < goal_rotate_ticks + goal_translate_ticks)) {
            MSGE("openloop - translation")
            if(goal_x < 0) TRANSLATE_LEFT
            else TRANSLATE_RIGHT
          } else {
            printf("(%s) openloop - aligned %.3f degrees, %.3f meters\n",TESTENV, goal_theta, goal_x);
            machine_status = APPROACH_GREEN_3;
            current_ticks = timer_ticks;
          }
          break;
        }

        /*
         * APPROACH_GREEN_3
         * feedback position(angular) control using CAM_btm
         */
        case APPROACH_GREEN_3:
        {
          #ifdef UNRELIABLE
          int idx_close = closest_ball(GREEN);
          int idx_far = furthest_green();

          float xg1 = green_x[idx_close];
          float zg1 = green_z[idx_close];
          float xg2 = green_x[idx_far];
          float zg2 = green_z[idx_far];
          #else
          float xg1 = green_x[0];
          float zg1 = green_z[0];
          float xg2 = green_x[1];
          float zg2 = green_z[1];
          #endif

          if(green_cnt < 2) {
            printf("(%s) APPROACH_GREEN : there are %d balls. phase out to APPROACH_GREEN\n", TESTENV, green_cnt);
            machine_status = APPROACH_GREEN;
            break;
          }

          float angular_ofs = RAD2DEG(atan((zg2 - zg1) / (xg2 - xg1)));

          if(!(timer_ticks%10)) printf("(%s) angle = %.4f\n", TESTENV, angular_ofs);

          if(angular_ofs < -1.6f) {
            MSGE("feedback - rotation CW")
            TURN_RIGHT_SLOW
          } else if(angular_ofs > 1.6f) {
            MSGE("feedback - rotation CCW")
            TURN_LEFT_SLOW
          } else {
            printf("(%s) finished alignment. angular deviation = %.4f deg\n",TESTENV, angular_ofs);
            machine_status = APPROACH_GREEN_4;
          }

          break;
        }

        case APPROACH_GREEN_4:
        {
          #ifdef UNRELIABLE
          int idx_close = closest_ball(GREEN);
          int idx_far = furthest_green();

          float xg1 = green_x[idx_close];
          float zg1 = green_z[idx_close];
          float xg2 = green_x[idx_far];
          float zg2 = green_z[idx_far];
          #else
          float xg1 = green_x[0];
          float xg2 = green_x[1];
          float zg1 = green_z[0];
          float zg2 = green_z[1];
          #endif

          float x_ofs = 0.5f * (xg1 + xg2);

          if(!(timer_ticks%10)) printf("(%s) X_offset = %.4f[m]\n", TESTENV, x_ofs);

          if(x_ofs > 0.01f) {
            MSGE("feedback - translation R")
            TRANSLATE_RIGHT
          } else if(x_ofs < -0.01f) {
            MSGE("feedback - translation L")
            TRANSLATE_LEFT
          } else {
            #ifndef UNRELIABLE
            machine_status = RELEASE;
            current_ticks = timer_ticks;
            #else
            machine_status = APPROACH_GREEN_5;
            machine_status = RELEASE;
            #endif
          }
          break;
        }

        #ifdef UNRELIABLE
        case APPROACH_GREEN_5:
        {
          MSGE("APPROACH_GREEN - 5 : Control over UNRELIABLE actuator")

          float xg1 = green_x[0];
          float xg2 = green_x[1];
          float zg1 = green_z[0];
          float zg2 = green_z[1];

          if(!(timer_ticks%10)) printf("(%.3f, %.3f), (%.3f, %.3f)\n", xg1, zg1, xg2, zg2);

          assert(DIST(xg1,zg1,xg2,zg2)>=0.05);
          float theta = RAD2DEG(atan((zg2-zg1)/(xg2-xg1)));
          if(!(timer_ticks%10)) printf("theta = %.3f\n", theta);
          if(theta < -1.5f) TURN_RIGHT_SLOW
          else if(theta > 1.5f) TURN_LEFT_SLOW
          else { current_ticks = timer_ticks; machine_status = RELEASE; }

          break;
        }
        #endif

        case RELEASE:
        {
          #ifndef LIDAR
          uint32_t goal_front_ticks = (uint32_t) (90.0f * (goal_z));

          if(timer_ticks-current_ticks < goal_front_ticks) {
            MSGE("RELEASE - go front")
            GO_FRONT
          } else if(timer_ticks - current_ticks < 100 + goal_front_ticks) {
            MSGE("RELEASE - roller_reverse")
            ROLLER_REVERSE
          } else {
	    printf("(%s) elapsed time = %.4f sec\n", TESTENV, 0.025 * timer_ticks);
            PANIC("RELEASE_TERMINATE : should have released 3 balls.")
          }
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

        default:
          assert(0);

      }


      /* Send control data */

      #ifdef MYRIO
      if(use_myrio) {
      size_t written = write(c_socket, data, sizeof(data));
      if(DEBUG) printf("%d bytes written\n", (int) written);
      }
      #endif

	    ros::Duration(DURATION).sleep();
	    ros::spinOnce();
      timer_ticks++;
    }

    close(c_socket);
    ros::shutdown();

    return -1;
}

#ifdef WEBCAM
/* camera_Callback : Updates position/ball_count of all colors */
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  /* Step 1. Fetch data from message */
  int b_cnt = position->size_b;
  blue_cnt = position->size_b;

  for(int i=0; i<b_cnt; i++) {
    // transform the matrix
    float x_pos = position->img_x_b[i];
    float y_pos = position->img_y_b[i];
    //float z_pos = sqrt(pow(position->img_z_b[i],2.0) - pow(position->img_x_b[i],2.0) - pow(position->img_y_b[i],2.0));
    float z_pos = position->img_z_b[i];
    //z_pos = position->img_z_b[i],2.0) - pow(position->img_x_b[i],2.0));

    /* TODO : transform (Camera coordinate)->(LLF) */
    blue_x[i] = x_pos - x_offset;
    blue_z[i] = z_pos - z_offset;
  }

  int r_cnt = position->size_r;
  red_cnt = position->size_r;

  for(int i=0; i<r_cnt; i++) {
    float x_pos = position->img_x_r[i];
    float y_pos = position->img_y_r[i];
   float z_pos = position->img_z_r[i];
    //float z_pos = sqrt(pow(position->img_z_r[i],2.0) - pow(red_x[i], 2.0) - pow(red_y[i], 2.0));
    //z_pos = sqrt(pow(position->img_z_r[i],2.0) - pow(position->img_x_r[i],2.0));

    /* TODO : transform (Camera coordinate)->(LLF) */
    red_x[i] = x_pos - x_offset;
    red_y[i] = (y_pos * cos(downside_angle) + z_pos * sin(downside_angle)) - y_offset;
    red_z[i] = (z_pos * cos(downside_angle) - y_pos * cos(downside_angle)) - z_offset;
    red_z[i] = z_pos - z_offset;
  }

  int g_cnt = position->size_g;
  green_cnt = position->size_g;

  for(int i=0; i<g_cnt; i++) {
    float x_pos = position->img_x_g[i];
    float y_pos = position->img_y_g[i];
    float z_pos = position->img_z_g[i]; //sqrt(pow(position->img_z_g[i],2.0) - pow(position->img_x_g[i],2.0));

    green_x[i] = x_pos - x_offset;
    green_z[i] = z_pos - z_offset;
  }

}

/* callback 2 */
void camera_Callback_top(const core_msgs::ball_position_top::ConstPtr& position)
{
  /* Step 1. Fetch data from message */
  int b_cnt_top = position->size_b;
  blue_cnt_top = position->size_b;

  for(int i=0; i<b_cnt_top; i++) {
    // transform the matrix
    float x_pos = position->img_x_b[i];
    float y_pos = position->img_y_b[i];
    float z_pos = position->img_z_b[i]; //sqrt(pow(position->img_z_b[i],2.0) - pow(position->img_x_b[i],2.0));

    /* TODO : transform (Camera coordinate)->(LLF) */
    blue_x_top[i] = x_pos - x_offset_top;
    blue_z_top[i] = z_pos - z_offset_top;
  }

  int r_cnt_top = position->size_r;
  red_cnt_top = position->size_r;

  for(int i=0; i<r_cnt_top; i++) {
    float x_pos = position->img_x_r[i];
    float y_pos = position->img_y_r[i];
    float z_pos = position->img_z_r[i]; //sqrt(pow(position->img_z_r[i],2.0) - pow(position->img_x_r[i],2.0));

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
    float z_pos = position->img_z_g[i]; //sqrt(pow(position->img_z_g[i],2.0) - pow(position->img_x_g[i],2.0));

    /* TODO : transform (Camera coordinate)->(LLF) */
    green_x_top[i] = x_pos - x_offset;
    green_z_top[i] = z_pos - z_offset;
  }
}

void camera_Callback_counter(const core_msgs::roller_num::ConstPtr& cnt)
{
  int shift = cnt->size_b;

  if(shift && !return_mode) {
    printf("(%s) dupACKcnt = %d\n", TESTENV, (int) ++dupACKcnt);
    if(dupACKcnt >= 3) {  // 3 Duplicate ACK
      if(machine_status == SEARCH || !return_mode) {
        machine_status = LIDAR_RETURN;
        return_mode = 1;
      }
    }
  }
  else
    dupACKcnt = 0;

}


bool ball_in_range(enum color ball_color) {

  if(ball_color == BLUE) {
    for(int i=0; i<blue_cnt; i++) {
      //printf("blue[%d] = (%.2f, %.2f)\n", i, blue_x[i], blue_z[i]);
      if(fabs(blue_x[i])<=0.034 && blue_z[i] <= 0.3)
        return true;
    }
    return false;

  } else if(ball_color == RED) {
    return red_in_range();

  } else {
    return false;
  }
}

bool red_in_range() {
  for(int i=0; i<red_cnt; i++) {
    if(fabs(red_x[i])<0.25 && red_z[i] <= 0.3)
      return true;
  }
  return false;
}

/*
 * leftmost green()
 * return leftmost blue visible in sight
 * return -1 if invisible
 */
int leftmost_green() {
  if(!green_cnt)
    return -1;

  float xpos = green_x[0];
  int min_idx = 0;

  for(int i=0; i<green_cnt; i++) {
    if(green_x[i] <= xpos) {
      xpos = green_x[i];
      min_idx = i;
    }
  }
  return min_idx;
}

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
 * return leftmost blue visible in sight
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
 */
int target_blue(int policy) {
  switch(policy) {
    case LEFTMOST:
      return leftmost_blue();
    case CENTERMOST:
      return centermost_blue();
    case CLOSEST:
      return closest_ball(BLUE);
    default:
      PANIC("Undefined ball policy");
  }
}

/*
 * target_blue(int policy)
 * policy : LEFTMOST(0), CENTERMOST(1), CLOSEST(2)
 */
int target_blue_top(int policy) {
  switch(policy) {
    case LEFTMOST:
      return leftmost_blue_top();
    case CENTERMOST:
      return -1;
    case CLOSEST:
      return closest_blue_top();
    default:
      PANIC("Undefined ball policy");
  }
}


/*
 * centermost blue()
 * return index of centermost blue ball amongst visible ones
 * -1 if no blue ball is in sight
 */
int centermost_blue() {
  if(!blue_cnt)
    return -1;

  float xpos_abs = fabs(blue_x[0]);
  int min_idx = 0;

  for(int i=0; i<blue_cnt; i++) {
    if(fabs(blue_x[i]) < xpos_abs) {
      xpos_abs = fabs(blue_x[i]);
      min_idx = i;
    }
  }
  return min_idx;
}

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

int closest_ball(enum color ball_color) {
  switch(ball_color) {
    case BLUE:
    {
      int result_idx = -1;

      if(!blue_cnt)
        return -1;

      float front_dist = blue_z[0];

      for(int i=0; i<blue_cnt; i++){
        if(blue_z[i] <= front_dist){
          front_dist = blue_z[i];
          result_idx = i;
        }
      }

      return result_idx;
    }
    case RED:
    {
      int result_idx = -1;

      if(!red_cnt)
        return -1;

      float front_dist = red_z[0];
      for(int i=0; i<red_cnt; i++){
        if(red_z[i] <= front_dist){
          front_dist = red_z[i];
          result_idx = i;
        }
      }

        return result_idx;
      break;
    }
    case GREEN:
    {
      int result_idx = -1;

      if(!green_cnt)
        return -1;
      float front_dist = green_z[0];
      for(int i=0; i<green_cnt; i++) {
        if(green_z[i] <= front_dist) {
          front_dist = green_z[i];
          result_idx = i;
        }
      }
      return result_idx;
    }
    default:
      { return -1; }
  }
}

/*
 * furthest_green()
 * Traverse through green_z
 * returns INDEX of maximum dist Z
 *
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
