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

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1" // myRIO ipadress
#define MYRIO
#undef MYRIO

#define DEBUG 0 
#define PERIOD 10

#define ANGULAR_RANGE 30

#define TURN_RIGHT { data[2] = 1; }
#define TURN_LEFT { data[3] = 1; }
#define GO_FRONT { data[0] = 1; }
#define GO_BACK { data[1] = 1; }
#define ROLLER_ON { data[4] = 1; }
#define ROLLER_REVERSE { data[5] = 1; }

#define TURN_BACK { data[6] = 1; }

/* For timer features */
uint32_t timer_ticks = 0;
uint32_t current_ticks = 0;

/* State variable declaration */
enum status {
  /* Moving around */
  SEARCH, APPROACH, RED_AVOIDANCE,
  /* Turn on roller only for this state */
  COLLECT,
  /* Return to goal pos */
  SEARCH_GREEN, APPROACH_GREEN, RELEASE
};

enum color {
  NONE, BLUE, RED, GREEN
};

enum actions {
  TURN_LEFT_, TURN_RIGHT_, GO_FRONT_, GO_BACK_
};

/* State of our machine = SEARCH phase by default */
enum status machine_status = SEARCH;
enum color closest_ball = NONE;

/* Number of balls holding */
int ball_cnt = 0; 

#ifdef NOT_REACHED
/* Synchronization primitives and Lidar */
boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int near_ball;
int action;
#endif

/* Ball detection */

/* Blue balls */
int ball_number;
float ball_X[20];
float ball_Y[20];

/* Red balls */
int ball_number_r;
float ball_X_r[20];
float ball_Y_r[20];

/* Unused in this scheme */
float ball_distance[20];

/* Track the closest ball. */
int target_ball;  // index of target ball

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];

/* Function prototypes */
void dataInit();
void find_ball();
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan);
void camera_Callback(const core_msgs::ball_position::ConstPtr& position);
int target(size_t ball_cnt);

bool red_in_range();

#define TIMEOUT 20
#define MAXSIZE 20
#define THRESH 0.3f

const std::string cond[] = { "SEARCH", "APPROACH", "RED_AVOIDANCE", "COLLECT", "SEARCH_GREEN", "APPROACH_GREEN", "RELEASE" };

/* Main routine */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    #ifdef UNUSED
      ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    #endif
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position_top", 1000, camera_Callback);

		dataInit();

    #ifdef MYRIO
    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
      printf("Failed to connect\n");
      close(c_socket);
      return -1;
    }
    #endif
    while(ros::ok){
      /* This part is for TCP connection teardown */
      if(timer_ticks>TIMEOUT*PERIOD){
        /* server side close() */
        if(timer_ticks < (TIMEOUT+1)*PERIOD+1){
          data[7] = 1;
          write(c_socket, data, sizeof(data)) ;
       } else { 
          /* client side close() */
          close(c_socket);
          ros::shutdown();
          return 0;
        }
      } else {

      dataInit();

      switch(machine_status) {
        case SEARCH:
          // during search phase, simply turn right
          TURN_RIGHT
          if(DEBUG && timer_ticks%PERIOD==0) std::cout << cond[machine_status] << std::endl; 
          break;
        case APPROACH:
          // approaching phase naively goes front, until ball is in range.
          GO_FRONT
          if(DEBUG && timer_ticks%PERIOD==0) std::cout << cond[machine_status] << std::endl; 
          break;
        case RED_AVOIDANCE:
          if(DEBUG && timer_ticks%PERIOD==0) std::cout << cond[machine_status] << std::endl; 
          break;
        case COLLECT:
          GO_FRONT ROLLER_ON
          if(DEBUG && timer_ticks%PERIOD==0) std::cout << cond[machine_status] << std::endl; 
          break;
        case SEARCH_GREEN:
          if(DEBUG && timer_ticks%PERIOD==0) std::cout << cond[machine_status] << std::endl; 
          break;
        case APPROACH_GREEN:
          break;
        case RELEASE: // This mode needs some senitel code for backup
          break;
        default:
          break;
      }

    #ifdef MYRIO
    size_t written = write(c_socket, data, sizeof(data));
    if(DEBUG)
      printf("%d bytes written\n", (int) written);
    #endif

      }
	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
      timer_ticks++;
    }
    close(c_socket);
    ros::shutdown();
    return 0;
}


/* 
 * camera_Callback : Updates position/ball_count of all colors
 * TODO : define separate callbacks for each colors(BLUE, RED, GREEN)
 * (ad-hoc) For now, we safely assume that there are blue balls only.
 */
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    int count = position->size_b;
    ball_number = count;
    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_x_b[i];
        ball_Y[i] = position->img_z_b[i];
        #ifndef DEBUG
          std::cout << "degree : "<< ball_degree[i];
          std::cout << "   distance : "<< ball_distance[i]<<std::endl;
        #endif
		ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_X[i];
    }
    if(count && DEBUG)
      printf("ball[0] : (X=%f, Y=%f)\n", ball_X[0], ball_Y[0]);
    switch(machine_status) {
      case SEARCH:
        if(target(count) != -1)
          machine_status = APPROACH;
        break;
      case APPROACH:
        if(target(count) == -1)
          machine_status = SEARCH;
        else if(ball_Y[target(count)] < THRESH)
          machine_status = COLLECT;
        break;
      case RED_AVOIDANCE:
      case COLLECT:
      {
        int idx = target(count);
        if(idx == -1) {
          printf("Consumed ball. Ball count = %d\n", ++ball_cnt);
          machine_status = SEARCH;
        } else if(ball_Y[idx] >= THRESH) {
          printf("Consumed ball. Ball count = %d\n", ++ball_cnt);
          machine_status = SEARCH;
        }
        break;
      }
      case SEARCH_GREEN:
      case APPROACH_GREEN:
      case RELEASE:
      default:
        break;
    }
}

int target(size_t ball_cnt) {
  // No blue ball in range : should keep spinning
  if(!ball_cnt)
    return -1;

  // There is a blue ball : should align ball to center
  int k = 0;
  float x_range = fabs(ball_X[0]);

  for(int i = 0 ; i < ball_cnt; i++) {
    if(x_range > fabs(ball_X[i])){
      x_range = fabs(ball_X[i]);
      k = i;
    }
  }
  if(RAD2DEG(atan(x_range/ball_Y[k])) < ANGULAR_RANGE) 
    return k;
  else
    return -1;
}

void find_ball()
{
	data[20]=1;
}

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

bool red_in_range() {

  bool result = false;
  return result;
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
