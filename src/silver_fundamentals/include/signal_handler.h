#ifndef SIGNAL_HANDLER
#define SIGNAL_HANDLER
#include <ros/ros.h>
#include "create_fundamentals/DiffDrive.h"
// #include <DriveSrv.h>

struct TimingStats {
    // accumulators (in seconds)
    double meas_time    = 0.0;
    double weight_time  = 0.0;
    double resamp_time  = 0.0;
    double odo_time     = 0.0;
    double viz_time     = 0.0;
    double drive_time   = 0.0;
    // counters
    int loops          = 0;
};

extern TimingStats stats;

void initSignalHandler();
bool isShutdownRequested();

#endif