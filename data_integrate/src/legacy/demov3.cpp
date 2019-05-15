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
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include "opencv2/opencv.hpp"
#include "util.hpp"


#define WEBCAM
#define MYRIO
#define DISTANCE_TICKS 150
#undef MYRIO
/* State of our machine = SEARCH phase by default */
enum status machine_status = SEARCH;
enum status recent_status = SEARCH;

/* Number of balls holding */
int ball_cnt = 0;

#ifdef LIDAR
/* Synchronization primitives and Lidar */
boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int near_ball;
int action;
#endif

#ifdef WEBCAM
/* Blue balls */
int blue_cnt;
float blue_x[20];
float blue_y[20];
float blue_z[20];

/* Red balls */
int red_cnt;
float red_x[20];
float red_y[20];
float red_z[20];
#endif

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];

float x_offset, y_offset, z_offset;
float downside_angle;

bool red_phase2 = false;

// CAM01 : default x_ofs = -0.013, z_ofs = 0.130

/* Main routine */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    /* Argument parsing */
    int tag;

    char x_offset_[5];
    char y_offset_[5];
    char z_offset_[5];
    char downside_angle_[6];

    int flag = 0;

    memset(x_offset_, 0, 6);
    memset(y_offset_, 0, 6);
    memset(z_offset_, 0, 6);
    memset(downside_angle_, 0, 6);

    while((tag = getopt(argc, argv, "x:y:z:d:")) != -1) {
      switch(tag) {
        case 'x':
          flag |= 0x8;
          memcpy(x_offset_, optarg, strlen(optarg));
          break;
        case 'y':
          flag |= 0x4;
          memcpy(y_offset_, optarg, strlen(optarg));
          break;
        case 'z':
          flag |= 0x2;
          memcpy(z_offset_, optarg, strlen(optarg));
          break;
        case 'd':
          flag |= 0x1;
          if(strlen(optarg) > 6) {
            printf("Invalid characters or too large angle value. Please try smaller values\n");
            return -1;
          }
          memcpy(downside_angle_, optarg, strlen(optarg));
          break;
      }
    }

    if(!flag) {
      printf("[Usage] \n");
      printf("rosrun data_integrate demo_simple -x <X-offset> -y <Y-offset> -z <Z-offset> -d <camera angle>\n");
      return -1;
    }

    if(!(flag & 0x1) && 0) {
      printf("Missing -d option : downside angle necessary!\n");
      return -1;
    }

    x_offset = atof(x_offset_);
    y_offset = atof(y_offset_);
    z_offset = atof(z_offset_);
    downside_angle = RAD(atof(downside_angle_));

    printf("(demo-simple) start\n");
    printf("(demo-simple) camera offset : x=%.3f, y=%.3f, z=%.3f [m]\n", x_offset, y_offset, z_offset);
    printf("(demo-simple) downside angle : %.2f [deg] \n", RAD2DEG(downside_angle));

    #ifdef LIDAR
      ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    #endif

    #ifdef WEBCAM
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position_top", 1000, camera_Callback);
    #endif

		dataInit();

    #ifdef MYRIO
    printf("(demo-simple) Connecting to %s:%d\n", IPADDR, PORT);
    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
      printf("(demo-simple) Failed to connect\n");
      close(c_socket);
      return -1;
    }
    printf("(demo-simple) Connected to %s:%d\n", IPADDR, PORT);
    #endif

    printf("(demo-simple) Entering main routine...\n");
    printf("(demo-simple) state = SEARCH\n");

    while(ros::ok){
      dataInit();

      if(recent_status != machine_status)
        std::cout << "(demo-simple) state = " << cond[(recent_status = machine_status)] << std::endl;

      /* switching between states */
      switch(machine_status) {
        case SEARCH:
        {
          int target_b = centermost_blue();
          
          //target_b = closest_ball(BLUE);

          if(target_b < 0)
            TURN_RIGHT
          else {
            if(blue_x[target_b] > 0.1) TURN_RIGHT
            else if(blue_x[target_b] < -0.1) TURN_LEFT
          }

          break;
        }
        case APPROACH:
        {
          GO_FRONT
          break;
        }
        case RED_AVOIDANCE:
        {
          if(!red_phase2) TURN_RIGHT
          else {
            GO_FRONT
            if(timer_ticks - current_ticks > DISTANCE_TICKS) {
              red_phase2 = false;
              machine_status = SEARCH;
          }
        }
          break;
        }
        case COLLECT:
        {
          ROLLER_ON

          int target = closest_ball(BLUE);
          if(target == -1){
            //machine_status = SEARCH;
            //printf("(demo-simple) collected ball. ball_count = %d\n", ++ball_cnt);
          }
          else{
            float xpos = blue_x[target];
            float zpos = blue_z[target];

            if(xpos > 0.033) {TURN_RIGHT ROLLER_ON}
            else if(xpos < -0.033) {TURN_LEFT ROLLER_ON}
            else { GO_FRONT ROLLER_ON }
            }
          break;
        }
        case SEARCH_GREEN:
        case APPROACH_GREEN:
        case RELEASE:
        {
          PANIC("NotImplementedError at SEARCH_GREEN");
        }

        default:
          assert(0);

      }


      /* Send control data */

      #ifdef MYRIO
      size_t written = write(c_socket, data, sizeof(data));
      if(DEBUG)
        printf("%d bytes written\n", (int) written);
      #endif

	    ros::Duration(0.025).sleep();
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
    float z_pos = sqrt(pow(position->img_z_b[i],2.0) - pow(position->img_x_b[i],2.0) - pow(position->img_y_b[i],2.0));
    z_pos = sqrt(pow(position->img_z_b[i],2.0) - pow(position->img_x_b[i],2.0));

    /* TODO : transform (Camera coordinate)->(LLF) */
    blue_x[i] = x_pos - x_offset;
//    blue_y[i] = (y_pos * cos(downside_angle) + z_pos * sin(downside_angle)) - y_offset;
//    blue_z[i] = (z_pos * cos(downside_angle) - y_pos * cos(downside_angle)) - z_offset;
    blue_z[i] = z_pos - z_offset;
  }

  int r_cnt = position->size_r;
  red_cnt = position->size_r;

  for(int i=0; i<r_cnt; i++) {
    float x_pos = position->img_x_r[i];
    float y_pos = position->img_y_r[i];
    float z_pos = sqrt(pow(position->img_z_r[i],2.0) - pow(red_x[i], 2.0) - pow(red_y[i], 2.0));
    z_pos = sqrt(pow(position->img_z_r[i],2.0) - pow(position->img_x_r[i],2.0));

    /* TODO : transform (Camera coordinate)->(LLF) */
    red_x[i] = x_pos - x_offset;
    red_y[i] = (y_pos * cos(downside_angle) + z_pos * sin(downside_angle)) - y_offset;
    red_z[i] = (z_pos * cos(downside_angle) - y_pos * cos(downside_angle)) - z_offset;
    red_z[i] = z_pos - z_offset;
  }

  /* Step 2. state decision and transition */
   switch(machine_status) {
      case SEARCH:
      {
        int target = centermost_blue();
        if(target >= 0) {
          float xpos = blue_x[target];
          float zpos = blue_z[target];

          if(fabs(xpos) < 0.05)
            machine_status = APPROACH;
        }
        break;
      }

      case APPROACH:
      {
        int target = centermost_blue();
        float xpos, zpos;

        if(target >= 0) {
          xpos = blue_x[target];
          zpos = blue_z[target];
        }

        if(red_in_range()) {
          machine_status = RED_AVOIDANCE;
        } else if(fabs(xpos)>=0.05 || target<0) {
          machine_status = SEARCH;
        } else if(ball_in_range(BLUE)) {
          machine_status = COLLECT;
        }
        break;
      }

      case RED_AVOIDANCE:
      {
        if(red_in_range()) {
          current_ticks = timer_ticks;
          red_phase2 = false;
        } else {
          red_phase2 = true;
        }
        break;
      }
      case COLLECT:
      {
        int target = closest_ball(BLUE);
        //printf("target = %d\n", target);

        if(target = -1) {
          machine_status = SEARCH;
          printf("(demo-simple) Got the ball. Ball count = %d\n", ++ball_cnt);
        } else if(blue_z[target] > 0.5) {
          //machine_status = SEARCH;
          //printf("(demo-simple) Got the ball. Ball count = %d\n", ++ball_cnt);
        } else if(!ball_in_range(BLUE)) {
          //machine_status=SEARCH;
        }
        if(red_in_range()) machine_status = RED_AVOIDANCE;

        break;
      }
      case SEARCH_GREEN:
      case APPROACH_GREEN:
      case RELEASE:
      default:
        break;
    }
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
    if(fabs(red_x[i])<0.11 && red_z[i] <= 0.25)
      return true;
  }
  return false;
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

int closest_ball(enum color ball_color) {
  switch(ball_color) {
    case BLUE:
    {
      int result_idx = -1;

      if(!blue_cnt)
        return -1;

      float front_dist = blue_z[0];

      for(int i=0; i<blue_cnt; i++){
        if(blue_z[i] < front_dist){
          front_dist = blue_z[i];
          result_idx = i;
        }
      }

      return result_idx;

      if(fabs(blue_x[result_idx]) < 0.033)
        return result_idx;
      else{
        return -1;
      }
    }
    case RED:
    {
      int result_idx = -1;

      if(!red_cnt)
        return -1;

      float front_dist = red_z[0];
      for(int i=0; i<red_cnt; i++){
        if(red_z[i] < front_dist){
          front_dist = red_z[i];
          result_idx = i;
        }
      }

      if(fabs(red_x[result_idx] < 0.15) || 1) {
        return result_idx;
      } else {
        return -1;
      }
      break;
    }
    default:
      { return -1; }
  }
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


#ifdef NOT_REACHED

// LIDAR is UNUSED for this project
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
  		map_mutex.lock();

     int count = scan->scan_time / scan->time_increment;
     lidar_size=count;
     for(int i = 0; i < count; i++)
     {
         lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
         lidar_distance[i]=scan->ranges[i];
     }
  		map_mutex.unlock();
  }


  /* Checks for data subscription */
	  for(int i = 0; i < lidar_size; i++)
    {
	    std::cout << "degree : "<< lidar_degree[i];
	    std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
	  }
		for(int i = 0; i < ball_number; i++)
		{
			std::cout << "ball_X : "<< ball_X[i];
			std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
		}

  /* Sample code */
		////////////////////////////////////////////////////////////////
		// // 자율 주행을 예제 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
		// ////////////////////////////////////////////////////////////////
		// dataInit();
		// for(int i = 0; i < lidar_size-1; i++)
		// 	    {
		// 		if(lidar_distance[i]<lidar_distance[i+1]){lidar_obs=i;}
		// 		else if(lidar_distance[i]==lidar_distance[i+1]){lidar_obs=i;}
		// 		else {lidar_obs=i+1;}
		// 	    }
		// if(ball_number==0 || lidar_obs<0.3)
		// {
		// 		find_ball();
		// }
		// else
		// {
		// 	for(int i = 0; i < ball_number-1; i++)
		// 	    {
		// 		if(ball_distance[i]<ball_distance[i+1]){near_ball=i;}
		// 		else if(ball_distance[i]==ball_distance[i+1]){near_ball=i;}
		// 		else {near_ball=i+1;}
		// 	    }
		// 	if(ball_distance[near_ball]<0.1){data[4]=0; data[5]=0; data[21]=0;}
		// 	else
		// 	{
		// 		data[20]=1;
		// 		if(ball_X[near_ball]>0){data[4]=1;}  else{data[4]=-1;}
		// 		if(ball_Y[near_ball]>0){data[5]=1;}  else{data[5]=-1;}
		// 	}
		// }

		//자율 주행 알고리즘에 입력된 제어데이터(xbox 컨트롤러 데이터)를 myRIO에 송신(tcp/ip 통신)
	      // for (int i = 0; i < 24; i++){
	      // printf("%f ",data[i]);
				//
	      // }
	      // printf("\n");
			// printf("%d\n",action);


#endif
