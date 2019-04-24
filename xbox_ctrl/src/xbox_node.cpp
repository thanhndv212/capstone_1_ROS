#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>

extern "C" {
	#include "xbox_ctrl/gamepad.h"
}

using namespace std;

#define PORT 4000
#define IPADDR "10.42.0.133"
//#define IPADDR "127.0.0.1"
#define resol_x 640
#define resol_y 480

#define RANGE_DEAD1 50
#define RANGE_STATE_CHANGE 300

static const char* button_names[] = {
	"d-pad up",
	"d-pad down",
	"d-pad left",
	"d-pad right",
	"start",
	"back",
	"left thumb",
	"right thumb",
	"left shoulder",
	"right shoulder",
	"???",
	"???",
	"A",
	"B",
	"X",
	"Y"
};

int len;
int n;
float data[24];

static int line = 0;

static void logevent(const char* format, ...) {
	va_list va;

	move(9 + line, 0);
	clrtoeol();

	move(8 + line, 0);
	clrtoeol();

	va_start(va, format);
	vwprintw(stdscr, format, va);
	va_end(va);

	if (++line == 14) {
		line = 0;
	}
}

static void update(GAMEPAD_DEVICE dev) {

	float lx, ly, rx, ry;
	move(dev, 0);

	if (!GamepadIsConnected(dev)) {
		printw("%d) n/a\n", dev);
		return;
	}

	GamepadStickNormXY(dev, STICK_LEFT, &lx, &ly);
	GamepadStickNormXY(dev, STICK_RIGHT, &rx, &ry);

	printw("%d) L:(%+.3f,%+.3f :: %+.3f,%+.3f) R:(%+.3f, %+.3f :: %+.3f,%+.3f) LT:%+.3f RT:%+.3f ",
	dev,
	lx, ly,
	GamepadStickAngle(dev, STICK_LEFT),
	GamepadStickLength(dev, STICK_LEFT),
	rx, ry,
	GamepadStickAngle(dev, STICK_RIGHT),
	GamepadStickLength(dev, STICK_RIGHT),
	GamepadTriggerLength(dev, TRIGGER_LEFT),
	GamepadTriggerLength(dev, TRIGGER_RIGHT));
	printw("U:%d D:%d L:%d R:%d ",
	GamepadButtonDown(dev, BUTTON_DPAD_UP),
	GamepadButtonDown(dev, BUTTON_DPAD_DOWN),
	GamepadButtonDown(dev, BUTTON_DPAD_LEFT),
	GamepadButtonDown(dev, BUTTON_DPAD_RIGHT));
	printw("A:%d B:%d X:%d Y:%d Bk:%d St:%d ",
	GamepadButtonDown(dev, BUTTON_A),
	GamepadButtonDown(dev, BUTTON_B),
	GamepadButtonDown(dev, BUTTON_X),
	GamepadButtonDown(dev, BUTTON_Y),
	GamepadButtonDown(dev, BUTTON_BACK),
	GamepadButtonDown(dev, BUTTON_START));
	printw("LB:%d RB:%d LS:%d RS:%d\n",
	GamepadButtonDown(dev, BUTTON_LEFT_SHOULDER),
	GamepadButtonDown(dev, BUTTON_RIGHT_SHOULDER),
	GamepadButtonDown(dev, BUTTON_LEFT_THUMB),
	GamepadButtonDown(dev, BUTTON_RIGHT_THUMB));
}
uint8_t tchar = 0xFF;

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

int flag_get_ball = 0;

int main(int argc, char **argv) {
	ros::init(argc, argv, "XboxCtrl");
	ros::start();

	ros::NodeHandle nh;
	ros::Publisher msg_pub = nh.advertise<std_msgs::Float32MultiArray>("/xbox_ctrl", 1);
	std_msgs::Float32MultiArrayPtr msg_ctrl_;
	msg_ctrl_.reset(new std_msgs::Float32MultiArray);

	int ch, i, j, k;
	float lx, ly, rx, ry;
	initscr();
	cbreak();
	noecho();
	timeout(1);

	GamepadInit();

	while ((ch = getch()) != 'q') {
		GamepadUpdate();

		double dev = GAMEPAD_0;
		GAMEPAD_DEVICE _dev = static_cast<GAMEPAD_DEVICE>(dev);
		GamepadStickNormXY(_dev, STICK_LEFT, &lx, &ly);
		GamepadStickNormXY(_dev, STICK_RIGHT, &rx, &ry);

		data[0] = lx*data[3];
		data[1] = ly*data[3];
		data[2] = GamepadStickAngle(_dev, STICK_LEFT);
		data[3] = GamepadStickLength(_dev, STICK_LEFT);
		data[4] = rx*data[7];
		data[5] = ry*data[7];
		data[6] = GamepadStickAngle(_dev, STICK_RIGHT);
		data[7] = GamepadStickLength(_dev, STICK_RIGHT);
		data[8] = GamepadTriggerLength(_dev, TRIGGER_LEFT);
		data[9] = GamepadTriggerLength(_dev, TRIGGER_RIGHT);
		data[10] = GamepadButtonDown(_dev, BUTTON_DPAD_UP);
		data[11] = GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
		data[12] = GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
		data[13] = GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
		data[14] = GamepadButtonDown(_dev, BUTTON_A);
		data[15] = GamepadButtonDown(_dev, BUTTON_B);
		data[16] = GamepadButtonDown(_dev, BUTTON_X);
		data[17] = GamepadButtonDown(_dev, BUTTON_Y);
		data[18] = GamepadButtonDown(_dev, BUTTON_BACK);
		data[19] = GamepadButtonDown(_dev, BUTTON_START);
		data[20] = GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
		data[21] = GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
		data[22] = GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
		data[23] = GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);

		msg_ctrl_->data.clear();
		msg_ctrl_->data.insert(msg_ctrl_->data.end(), data, data+24);
		msg_pub.publish(msg_ctrl_);
		ros::Duration(0.05).sleep();

		if (ch == 'r') {
			for (i = 0; i != GAMEPAD_COUNT; ++i) {
				GamepadSetRumble(static_cast<GAMEPAD_DEVICE>(i), 0.25f, 0.25f);
			}
		}

		update(GAMEPAD_0);

		{
			GAMEPAD_DEVICE _i = static_cast<GAMEPAD_DEVICE>(0);
			if (GamepadIsConnected(_i)) {
				for (j = 0; j != BUTTON_COUNT; ++j) {
					GAMEPAD_BUTTON _j = static_cast<GAMEPAD_BUTTON>(j);
					if (GamepadButtonTriggered(_i, _j)) {
						logevent("[%d] button triggered: %s", i, button_names[j]);
					} else if (GamepadButtonReleased(_i, _j)) {
						logevent("[%d] button released:  %s", i, button_names[j]);
					}
				}
				for (j = 0; j != TRIGGER_COUNT; ++j) {
					GAMEPAD_TRIGGER _j = static_cast<GAMEPAD_TRIGGER>(j);
					if (GamepadTriggerTriggered(_i, _j)) {
						logevent("[%d] trigger pressed:  %d", i, j);
					} else if (GamepadTriggerReleased(_i, _j)) {
						logevent("[%d] trigger released: %d", i, j);
					}
				}
				for (j = 0; j != STICK_COUNT; ++j) {
					GAMEPAD_STICK _j = static_cast<GAMEPAD_STICK>(j);
					for (k = 0; k != STICKDIR_COUNT; ++k) {
						GAMEPAD_STICKDIR _k = static_cast<GAMEPAD_STICKDIR>(k);
						if (GamepadStickDirTriggered(_i, _j, _k)) {
							logevent("[%d] stick direction:  %d -> %d", i, j, k);
						}
					}
				}
			}
		}

		move(6, 0);
		printw("(q)uit (r)umble");

		refresh();
	}

	ros::shutdown();

	endwin();

	return 0;
}
