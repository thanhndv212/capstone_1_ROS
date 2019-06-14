#include "core_msgs/ball_position.h"
#include "core_msgs/ball_position_top.h"
#include "core_msgs/roller_num.h"
#include "lidar/coor.h"

#ifndef UTIL_H
#define UTIL_H

#define RAD(deg) ((deg)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "172.16.0.1" // myRIO ipadress

// After this timeout, robot will give up remaining balls [sec]
#define TIMEOUT 12

#define DEBUG 0 
#define PERIOD 40

#define GO_FRONT { data[0] = -0.17f; data[1] = 1; data[2] = 1.75f; data[3] = 1; }
#define TURN_RIGHT_MID { data[0] = -0.06f; data[1] = -1; data[2] = -1.64f; data[3] = 1;  }
#define TURN_RIGHT { data[0] = 1; data[1] = -0.17f; data[2] = -0.17f; data[3] = 1;}
#define TURN_LEFT { data[0] = -1; data[1] = 0.15f; data[2] = 2.98f; data[3] = 1;}
#define ROLLER_ON { data[17] = 1; }
#define ROLLER_REVERSE { data[14] = 1; }
#define TRANSLATE_LEFT { data[16] = 1; }
#define TRANSLATE_RIGHT { data[15] = 1; }
#define TURN_RIGHT_SLOW { data[20] = 1; }
#define TURN_LEFT_SLOW { data[21] = 1; }
#define GO_FRONT_SLOW { data[23] = 1; }
#define TRANSLATE_LEFT_2x { data[8] = 1; }
#define TRANSLATE_RIGHT_2x { data[9] = 1; }
#define TRANSLATE_LEFT_3x { data[18] = 1; }
#define TRANSLATE_RIGHT_3x { data[19] = 1; }



#define MSGE(x) { if(!(timer_ticks%10)) std::cout << "(" << TESTENV << ") " << x << std::endl; }
#define PANIC(msg) { std::cout << "Kernel PANIC at " << msg << std::endl; assert(0); }

#define LEFTMOST 0
#define CENTERMOST 1
#define CLOSEST 2

/* For timer features */
static uint32_t timer_ticks = 0;
static uint32_t current_ticks = 0;

/* State variable declaration */
enum status {
  /* Initialize */
  INIT,

  /* Moving around */
  SEARCH, APPROACH, RED_AVOIDANCE,
  /* Turn on roller only for this state */
  COLLECT, COLLECT2,
  /* Return to goal pos */
  LIDAR_RETURN,
  SEARCH_GREEN,
  RELEASE
};

enum color {
  NONE, BLUE, RED, GREEN
};

enum actions {
  TURN_LEFT_, TURN_RIGHT_, GO_FRONT_, GO_BACK_
};

/* Function prototypes */
void dataInit();
void find_ball();
void lidar_Callback(const lidar::coor::ConstPtr& scan);
void camera_Callback(const core_msgs::ball_position::ConstPtr& position);
void camera_Callback_top(const core_msgs::ball_position_top::ConstPtr& position);
void camera_Callback_counter(const core_msgs::roller_num::ConstPtr& cnt);

bool red_in_range(); //if red ball position x= +-25 cm, z<=30 cm 일때 true , else false.
bool ball_in_range(enum color ball_color); 
             //if blue ball x= +-3.4 cm, z<=30 cm 일때 true, else false//
             //elseif red ball --> red_in_range//
             //else false//
                        
int closest_ball(enum color ball_color); //아래있는 카메라로 봤을때 가장 가까이 있는 ball color 를 가진 공에 대한 array의 index를 반환한다. 없으면 return -1//
int centermost_green(); //아래있는 카메라로 봤을때 가장 가운데에 있는 Green ball에 대한 array의 index를 반환한다. 없으면 return -1//
int leftmost_green(); //아래있는 카메라로 봤을때 가장 왼쪽에 있는 Green ball에 대한 array의 index를 반환한다. 없으면 return -1//
int rightmost_green(); //아래있는 카메라로 봤을때 가장 오른쪽에 있는 Green ball에 대한 array의 index를 반환한다. 없으면 return -1//
int leftmost_green_top(); //위에있는 카메라로 봤을때 가장 가운데에 있는 Green ball에 대한 array의 index를 반환한다. 없으면 return -1//
int closest_green(); //아래있는 카메라로 봤을때 가장 가까이 있는 Green ball에 대한 array의 index를 반환한다. 없으면 return -1//

int target_blue(int policy); // 제일 왼쪽, 가운데, 가까운 파란색 공의 index

int leftmost_blue(); //아래있는 카메라로 봤을때 가장 왼쪽에 있는 Blue ball에 대한 array의 index를 반환한다. 없으면 return -1//
int centermost_blue(); //아래있는 카메라로 봤을때 가장 가운데에 있는 Blue ball에 대한 array의 index를 반환한다. 없으면 return -1//
int leftmost_blue_top(); //위에있는 카메라로 봤을때 가장 왼쪽에 있는 Blue ball에 대한 array의 index를 반환한다. 없으면 return -1//
const std::string cond[] = {"INIT", "SEARCH", "APPROACH", "RED_AVOIDANCE", "COLLECT", "COLLECT2", "LIDAR_RETURN", "SEARCH_GREEN", "APPROACH_GREEN", "APPROACH_GREEN_2", "APPROACH_GREEN_3", "APPROACH_GREEN_4", "APPROACH_GREEN_5" , "RELEASE" };



#endif


