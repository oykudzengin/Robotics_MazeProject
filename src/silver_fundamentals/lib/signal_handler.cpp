#include <signal_handler.h>
#include <signal.h>
#include <atomic>


TimingStats stats;
static std::atomic_bool shutdown_requested(false);

void onSigint(int) {
    // ROS_INFO("SIGINT captured, setting shutdown flag");
    shutdown_requested = true;
}

void initSignalHandler() {
    signal(SIGINT, onSigint);
}


bool isShutdownRequested() {
    return shutdown_requested.load();
}