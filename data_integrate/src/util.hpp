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

#define GO_FRONT { data[0] = -0.17f; data[1] = 1; data[2] = 1.75f; data[3] = 1;  }
#define GO_BACK { data[0] = -0.06f; data[1] = -1; data[2] = -1.64f; data[3] = 1;  }
#define TURN_RIGHT { data[0] = 1; data[1] = -0.17f; data[2] = -0.17f; data[3] = 1; }
#define TURN_LEFT { data[0] = -1; data[1] = 0.15f; data[2] = 2.98f; data[3] = 1; }
#define ROLLER_ON { data[17] = 1; }
#define ROLLER_REVERSE { data[14] = 1; }

#define PANIC(msg) { std::cout << "Kernel PANIC : " << msg << std::endl; assert(0); }

/* For timer features */
static uint32_t timer_ticks = 0;
static uint32_t current_ticks = 0;

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

/* Function prototypes */
void dataInit();
void find_ball();
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan);
void camera_Callback(const core_msgs::ball_position::ConstPtr& position);

bool red_in_range();
bool ball_in_range(enum color ball_color);
int closest_ball(enum color ball_color);
int centermost_blue();

const std::string cond[] = { "SEARCH", "APPROACH", "RED_AVOIDANCE", "COLLECT", "SEARCH_GREEN", "APPROACH_GREEN", "RELEASE" };




#endif


