//#define START_ROLE 1 //1 - attacker

#pragma once

#define OTLADKA false
#define USE_MOTORS true
#define USE_DISPLAY true
#define USE_DRIBLER true
#define USE_FRONT_CAMERA false

#define STOP_DRIBLER_SPEED 200
#define MAX_DRIBLER_SPEED 40
#define GRAB_DRIBLER_SPEED 20
#define DRIBLER_REVERSE_KICK_SPEED 0
#define DRIBLER_KICKING_SPEED 0

#define POWER_SUPPLY 16
#define STANDART_POWER 13
#define POWER_K 1

#define ROBOT_MAX_X 80
#define ROBOT_MIN_X -80

#define ROBOT_MAX_Y 210
#define ROBOT_MIN_Y 10

#define CENTRAL_GATE_OUT 55
#define SIDE_GATE_OUT 58

#define CHANGE_SPEED_VECTOR_CONST 2

#define USE_BLUETOOTH 0

#define KP_GYRO_MOVING 1.0
#define KD_GYRO_MOVING 10.0
#define KI_GYRO_MOVING 0.01

#define KP_GYRO_STOPPED 1.0
#define KD_GYRO_STOPPED 10.0
#define KI_GYRO_STOPPED 0.01

#define BALL_DETECTION_LIGHTNESS 1500

#define BUTTON_MIN_PRESSING_TIME_MS 65

#define BALL_K1
#define BALL_K2
#define BALL_K3
#define BALL_K4

#define KICK_OFF_DURATION 1000

#define STANDART_ROBOTS_ROLE_FROM_FLASH 255

#define FAST_POINT_MOVEMENT_SPEED 255
#define SLOW_POINT_MOVEMENT_SPEED -255
