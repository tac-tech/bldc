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

static volatile bool drive_isRunning = false;
static volatile bool steering_comms_isRunning = false;
static volatile bool steering_controls_isRunning = false;
static volatile bool stop_now = true;

// Steering Thread mutexes + variables
static mutex_t steering_pos_mtx;
static bool is_mtx_setup = false;
static double steering_pos = -1.0;

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
    drive_isRunning = true;
    commands_printf("Drive thread started!");
    while (!stop_now){
        chThdSleepMilliseconds(50);
        timeout_reset();
    }
    commands_printf("Drive thread finished!");
    drive_isRunning = false;
}

static THD_FUNCTION(maverick_steering_comms_thread, arg){
    (void)arg;
    chRegSetThreadName("MAVERICK_STEERING_COMMS");
    steering_comms_isRunning = true;
    while (!stop_now){

        chMtxLock(&steering_pos_mtx);
        commands_printf("Comms Position: %f", steering_pos);
        chMtxUnlock(&steering_pos_mtx);

        chThdSleepMilliseconds(500);
        timeout_reset();
    }
    chMtxUnlockAll();
    steering_comms_isRunning = false;
}

static THD_FUNCTION(maverick_steering_controls_thread, arg){
    (void) arg;
    chRegSetThreadName("MAVERICK_STEERING_CONTROLS");
    spi_init();
    // commands_printf("Microseconds per clock ticks: %");
    steering_controls_isRunning = true;

    while(!stop_now) {
        // Get the current encoder value
        gen_response response = br10_getGeneralResponse();
        if (!response.crcCheck){
            // TODO: Add condition if crcCheck is bad
            // continue;
        }
        if (chMtxTryLock(&steering_pos_mtx)){
            steering_pos = response.position;
            commands_printf("Controls Position: %f", steering_pos);
            chMtxUnlock(&steering_pos_mtx);
        }
        // Calculate PID

        // Set motor

        // Thread sleep
        // TODO: replace with sleep unil to better control speed of cycle
        chThdSleepMilliseconds(100);
        timeout_reset();
    }
    steering_controls_isRunning = false;
    spi_deinit();
    chMtxUnlockAll();
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
            // Initialize the mutexes
            if (!is_mtx_setup) {
                chMtxObjectInit(&steering_pos_mtx);
                is_mtx_setup = true;
            }

            // Start threads
            chThdCreateStatic(maverick_steering_comms_thread_wa, sizeof(maverick_steering_comms_thread_wa),
                            NORMALPRIO, maverick_steering_comms_thread, NULL);
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
    while ( drive_isRunning && 
            steering_comms_isRunning &&
            steering_controls_isRunning){
        chThdSleepMilliseconds(10);
    }

    // Deinit mutexes
    // &steering_pos_mtx = NULL;
}