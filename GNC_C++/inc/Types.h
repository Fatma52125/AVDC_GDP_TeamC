#pragma once
#include <stdint.h>

//General buffer for storing msgs
extern char msgs[100];

//General temporal float 
extern float temp[10];

//Sampling Time
const float dt=0.02;

//Execution flag
extern uint8_t execute;

//Define some global variables
const float pi=3.14159265358979;
const float rad_to_deg=57.2957795130823;
const float deg_to_rad=0.0174532925199433;
const float eps=0.0000001;
const float max_val1=10000;
const float max_val2=1000;
const float g0=9.80665;

//Reasons definition
enum reasons{
    System_Checks=0,
    Coms=1,
    RC=2,
    Sensors=3,
    Control=4,
    Failsafe=5,
    Unknown=6
};
extern char reasons_msgs[7][10];

//Modes definition
enum modes{
    Acro=0,
    Stabilise=1,
    Alt_Hold=2,
    Pos_Hold=3,
    Guided=4,
    Mission=5,
    Track=6,
    Land=7,
    RTL=8
};
extern char modes_msgs[15][10];

//Failsafes definition
enum failsafes{
    Disarm_fs=0,
    Land_fs=1,
    Do_Nothing=2
};
extern char failsafe_msgs[3][10];

//Globals functions
extern uint8_t in_range(float y[], float max, float min);

extern void quat_to_dcm(float vec4[4],float vec9[9]);

extern void rpy_to_dcm(float vec3[3],float vec9[9]);

extern void quat_to_rpy(float vec4[4],float vec3[3]);

extern void quat_to_rpy(float vec4[4],float vec3_rad[3], float vec3[3]);

extern void quat_to_rates_rot(float vec4[4],float vec12[12]);