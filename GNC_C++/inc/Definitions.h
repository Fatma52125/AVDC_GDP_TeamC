#pragma once

//Core includes
#include <Core.h>

// //Class functions
#include <COMS.h>
#include <SYSTEM.h>
#include <DATALOG.h>
#include <CONTROL.h>
#include <MODEL.h>
#include <RECEIVER.h>
#include <PID.h>
#include <NAV_EKF.h>
#include <SENSORS.h>
#include <INPUTS.h>
#include <BATTERY.h>

//Non-class functions
#include <Param.h>
#include <Ellipsoid_Fitting.h>

//Extern general classes
extern COMS coms;
extern DATALOG datalog;
extern CONTROL control;
extern SYSTEM sys;
extern MODEL model;
extern RECEIVER receiver;
extern NAV_EKF nav_ekf;
extern SENSORS sensors;
extern INPUTS inputs;
extern BATTERY battery;