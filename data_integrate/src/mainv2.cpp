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
    signal(SIGSEGV, sigsegv_handler); 	//인덱스 참조 잘못했을 때 프로그램 terminate 방지

    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    /* Argument parsing */
    int tag;
//143~198: Argument 붙이는거 타임아웃 지정, 위치 보정(offset) 
    char x_offset_[5];
    char y_offset_[5];
    char z_offset_[5];
    char x_offset_top_[5];
    char z_offset_top_[5];
    char downside_angle_[6];
    char timeout_str[6];

    int flag = 0; //boolean variable to check argument content existence 
//포인터 안에 내용을 다 0으로 만든다 (initializing pointers)
    memset(x_offset_, 0, 6);
    memset(y_offset_, 0, 6);
    memset(z_offset_, 0, 6);
    memset(downside_angle_, 0, 6);


//argument parsing을 아용해 command에서 x, y, z offset과 use myrio, timeout을 설정할 수 있다.
//ex) rosrun data_integration data_integrate_node -x -0.013 -z 0.18 -T 60 -m
sets x_offset = 0.013, z_offset = 0.18, timeout = 60, and use_myrio = false.

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
	
//MYRIO가 define 되면, Myrio와 통신하기 위한 소켓을 생성하고, 초기화 시킨다 
    #ifdef MYRIO
    if(use_myrio) {
    printf("(%s) Connecting to %s:%d\n", TESTENV, IPADDR, PORT);
    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

//앞서 초기화한 세팅으로 myrio와 connect하는데,
//connect 실패하면 fail msg를 프린트 후 soket 닫고 -1 반환한다.
//connect 되면 connet msg를 프린트한다.
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

/* switching between states by machine_status */
      switch(machine_status) {
	// INIT phase
        case INIT:
        {
          MSGE("go front 2m first") //printf function that print debugging msg per 1/4s (function defined in header "util_rtn")
          GO_FRONT
	// 2s 동안 GO_FRONT 후 SEARCH phase로 넘어간다.
          if(timer_ticks - current_ticks >= 80) 
            machine_status = SEARCH;
          break;
        }
	//SEARCH phase      
        case SEARCH:
        {
	// POLICY : target selecting policy. 1. leftmost, 2. centermost, 3. closest ball
	// default POLICY : lestmost
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
            current_ticks = timer_ticks; //timer_ticks 현재 시간 (실시간), current_ticks 는 timer_tick에서 그때그때 받아온 시간 (실시간으로 안바뀜)
          }

          if(!red_phase2) TURN_LEFT
          else {
            GO_FRONT
//Return Mode (red avoidance)가 켜지면 그냥 Search_blue가 아닌 Search_green으로 돌아가게 해준다
            if(timer_ticks - current_ticks > DISTANCE_TICKS) { // DISTANCE_TICKS: 일정거리 전진하는 변수
              red_phase2 = false;
              machine_status = (return_mode)? SEARCH_GREEN : SEARCH;
          }
        }
          break;
        }
        case COLLECT:
        {
          ROLLER_ON
//멀리있는 왼쪽공 갈때 가까운 조금 오른쪽 공을 치고 가는 상황을 방지하기 위해서 가까운 공 먼저 먹게한다. 
          int target = closest_ball(BLUE);
          if(target==-1) target = leftmost_blue();

//
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
        
	//일정거리 앞으로 이동하면서 롤러 회전해서 공 진짜로 먹는거      
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

//웹캠만으로 멀리있는 초록공이 안보일때 Backup plan으로 라이다를 사용해서 Return하도록 하려고 코드를 짰지만,
//실제 데모장에서 초록공 Detection 문제가 없어서 라이다 코드는 사용하지 않았다. 
        case LIDAR_RETURN:
        {
          #ifndef LIDAR
          printf("(%s) LIDAR_RETURN : No LIDAR detected. Exitting.\n", TESTENV);
          machine_status = SEARCH_GREEN;

          #else //보고서 
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
        
//서치 그린안에 그린찾아서 릴리즈 까지 하는데 안에 가짜 서치가       
	case SEARCH_GREEN:
        {
          switch(green_cnt_top) {
            case 0:
		{ TURN_RIGHT_MID break; }
            case 1: //공이 하나만 보일때 그 공 보고 직진 (공이 멀리 있어서 하나만 보이는거 일 수도 있으니까)
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
            case 2: // 초록공 2개 동시에 보일때 실행
            default: //초록공이 3개 넘게 인식될때에 대한 Backup plan
            {
              float xg1 = green_x_top[0];
              float zg1 = green_z_top[0];
              float xg2 = green_x_top[1];
              float zg2 = green_z_top[1];

              float mid_x = 0.5 * (xg1 + xg2); //중점 찾기//
              float mid_z = 0.5 * (zg1 + zg2); 

              if(!(timer_ticks % 10)) printf(" (%.3f, %.3f), (%.3f, %.3f) \n", xg1, zg1, xg2, zg2);
//0.4초마다 프린트해주는거//
              float theta = RAD2DEG(atan((zg2-zg1)/(xg2-xg1))); //중점이랑 카메라 시야 사이의 각도//
//얼라인 할때 중점이 2미터 밖에 있으면 타깃 theta가 +_10도 안에 들어오게 align 하고 2미터 안에 들어올때까지 전진//

if(mid_z > 2.0f) {
  float target_theta = RAD2DEG(atan(mid_x/mid_z));
  if(target_theta > 10.0f) TURN_RIGHT
  else if(target_theta < -10.0f) TURN_LEFT
  else GO_FRONT


} 
//중점이 2미터 안에 있을때, theta가 10도에서 15도 사이 보다 작으면 
else {

              if(theta < -10.0f){
		if(theta < -15.0f) TURN_RIGHT //15도 보다 클때 빠르게 회전//
                else TURN_RIGHT_SLOW //10<theta<15 일때 느리게 회전: 정교한 alignment를 하기 위해//
                MSGE("turn_right_slow")

              } else if(theta > 10.0f) {
                if(theta > 15.0f) TURN_LEFT
		else TURN_LEFT_SLOW
                MSGE("turn_left_slow")
 
              } else if(mid_x < -0.02f) {
                if(mid_x < -0.10f) TRANSLATE_LEFT_3x
                else TRANSLATE_LEFT //_SLOW
              } else if(mid_x > 0.02f) {
                if(mid_x > 0.10f) TRANSLATE_RIGHT_3x
                else TRANSLATE_RIGHT //_SLOW
              } else {
                GO_FRONT
                MSGE("go_front")
              }

}
              if(0.5*(zg1+zg2) <= 0.8){ printf("goal distance = %.4f\n", 0.5*(zg1+zg2)); goal_z = 0.5*(zg1+zg2); 
		current_ticks = timer_ticks; machine_status = RELEASE; } //machine_status = APPROACH_GREEN;

		    //중점이 80cm 안으로 들어오면 state transition to "RELEASE" //
		    //거리를 goal_z에 저장한다//
            if(red_in_range())
              machine_status = RED_AVOIDANCE; //Search_Green 하면서 RED_AVOIDANCE 도 실행 

            break; //여기까지 초록공 Search & Alignment Case0: search, case1: approach, case 2: approach & align 
            }
          }
          break;
        }
	      
		      
		      

        case RELEASE:
        {
//웹캠만으로 멀리있는 초록공이 안보일때 Backup plan으로 라이다를 사용해서 Return하도록 하려고 코드를 짰지만,
//실제 데모장에서 초록공 Detection 문제가 없어서 라이다 코드는 사용하지 않았다.           
	  #ifndef LIDAR
          uint32_t goal_front_ticks = (uint32_t) (90.0f * (goal_z)); //Search_Green에서 정의된 goal_z 값을 사용하여 남은 직진 거리 정함

          if(timer_ticks-current_ticks < goal_front_ticks) { //직진 구간
            MSGE("RELEASE - go front")
            GO_FRONT
          } else if(timer_ticks - current_ticks < 100 + goal_front_ticks) { //도착한 뒤에 공 뱉기
            MSGE("RELEASE - roller_reverse")
            ROLLER_REVERSE
          } else { //시간 얼마나 걸렸는지 프린트하고 종료 
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
     /*myRIO에 array 보내는 코드*/
	    
      #ifdef MYRIO
      if(use_myrio) {
      size_t written = write(c_socket, data, sizeof(data)); //write 라는게 TCP/IP에 데이타를 보냄, 
	  
      if(DEBUG) printf("%d bytes written\n", (int) written);
      }
      #endif

	    ros::Duration(DURATION).sleep(); // 1/40초를 기다리게 한다 (time_tick 주기 맞춰주기 위함) //
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
//리포트 
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

// 위에 있는 카메라로 봤을때 가장 가까이 있는 Blue ball array 의 index를 반환한다. 없으면 return -1 //
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

// 아래있는 카메라로 봤을때 가장 가까이 있는 ball color 를 가진 공에 대한 array 의 index를 반환한다. 없으면 return -1 //
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
