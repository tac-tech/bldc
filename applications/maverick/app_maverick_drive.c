
#include "app_maverick_gen.h"
#include "maverick/app_maverick_drive.h"

void maverick_drive(arg){
    (void)arg;
    chRegSetThreadName("MAVERICK_DRIVE");
    drive_isRunning = true;
    commands_printf("Drive thread started!");
    while (!stop_now){
        chThdSleepMilliseconds(1000);
        timeout_reset();
    }
    commands_printf("Drive thread finished!");
    drive_isRunning = false;
}