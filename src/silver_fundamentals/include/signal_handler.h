#ifndef SIGNAL_HANDLER
#define SIGNAL_HANDLER
#include <ros/ros.h>
#include "create_fundamentals/DiffDrive.h"
// #include <DriveSrv.h>


void initSignalHandler();
bool isShutdownRequested();

#endif