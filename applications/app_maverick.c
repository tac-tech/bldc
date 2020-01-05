// Include system files
#include <ch.h>             // ChibiOS general file
#include <hal.h>            // ChibiOS hardware abstraction layer
#include <mc_interface.h>   // Motor Controller interface
#include <hw.h>             // Pin mapping on the hardware
#include <timeout.h>        // Used to reset timeout
#include <can.h>            // CAN interface
#include <commands.h>       // Basic commands, including print to console

#include "encoders/orbis_brd10/br10_sp.h"
#include "soft_spi.h"
#include "app_maverick.h"

// Private variables
static volatile app_configuration config;
static volatile maverick_motor_type motor_type;
static volatile bool is_running = false;
static volatile bool stop_now = true;

// Define Threads
static THD_FUNCTION(maverick_drive_thread, arg);
static THD_WORKING_AREA(maverick_drive_thread_wa, 2048);
static THD_FUNCTION(maverick_steering_comms_thread, arg);
static THD_WORKING_AREA(maverick_steering_comms_thread_wa, 2048);
static THD_FUNCTION(maverick_steering_controls_thread, arg);
static THD_WORKING_AREA(maverick_steering_controls_thread_wa, 2048);


static THD_FUNCTION(maverick_drive_thread, arg){
    (void)arg;
    chRegSetThreadName("MAVERICK_DRIVE");
    is_running = true;
    commands_printf("Drive thread started!");
    while (!stop_now){
        chThdSleepMilliseconds(50);
        timeout_reset();
    }
    commands_printf("Drive thread finished!");
    is_running = false;
}

static THD_FUNCTION(maverick_steering_comms_thread, arg){
    (void)arg;
    chRegSetThreadName("MAVERICK_STEERING_COMMS");
    is_running = true;
    while (!stop_now){

        chThdSleepMilliseconds(50);
        timeout_reset();
    }
    is_running = false;
}

static THD_FUNCTION(maverick_steering_controls_thread, arg){
    (void) arg;
    chRegSetThreadName("MAVERICK_STEERING_CONTROLS");
    spi_init();
    // commands_printf("Microseconds per clock ticks: %");
    is_running = true;
    while(!stop_now) {
        // Get the current encoder value
        detailed_status response = br10_getDetailedStatusResponse();
        commands_printf("Too Close: %d", response.isSignalAmplitudeHigh);
        commands_printf("Too Far: %d", response.isSignalAmplitudeLow);
        commands_printf("Temp: %d", response.isTempOutOfRange);
        commands_printf("Speed too high: %d", response.isSpeedHigh);
        commands_printf("Multiturn error: %d", response.isMultiturnError);
        commands_printf("Error: %d, Warning: %d", response.gen_response.isError, response.gen_response.isWarning);
        commands_printf("Position: %f \n", response.gen_response.position);
        
        // gen_response response = br10_getGeneralResponse();
        // commands_printf("Position: %f", response.position);
        
        // Calculate PID

        // Set motor

        // Thread sleep
        chThdSleepMilliseconds(100);
        timeout_reset();
    }
    is_running = false;
    spi_deinit();
}

void maverick_configure(app_configuration *conf){
    config = *conf;
    commands_printf("%d", config.controller_id);
    if (config.controller_id > min_motor_id){
        if (config.controller_id % 2 == 0){     // if even, then a steering motor
            motor_type = STEERING;
        } else {                                // if odd, then a drive motor
            motor_type = DRIVE;
        }
    } else {
        motor_type = NONE;
    }
}

void maverick_init(app_configuration *config){
    maverick_configure(config);
    switch(motor_type){
        case DRIVE:
            commands_printf("Drive");
            stop_now = false;
            chThdCreateStatic(maverick_drive_thread_wa, sizeof(maverick_drive_thread_wa),
                            NORMALPRIO, maverick_drive_thread, NULL);
            break;
        case STEERING:
            commands_printf("Steering");
            stop_now = false;
            // chThdCreateStatic(maverick_steering_comms_thread_wa, sizeof(maverick_steering_comms_thread_wa),
            //                 NORMALPRIO, maverick_steering_comms_thread, NULL);
            chThdCreateStatic(maverick_steering_controls_thread_wa, sizeof(maverick_steering_controls_thread_wa),
                            NORMALPRIO, maverick_steering_controls_thread, NULL);
            break;
        default:
            commands_printf("Default");
            break;
    }
}

void maverick_stop() {
    motor_type = NONE;
    stop_now = true;
    while (is_running){
        chThdSleepMilliseconds(10);
    }
}