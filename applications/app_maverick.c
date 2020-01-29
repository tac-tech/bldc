// Include system files
#include <ch.h>             // ChibiOS general file
#include <hal.h>            // ChibiOS hardware abstraction layer
#include <mc_interface.h>   // Motor Controller interface
#include <hw.h>             // Pin mapping on the hardware

#include "maverick/app_maverick_drive.h"
#include "maverick/app_maverick_steeringcomms.h"
#include "maverick/app_maverick_steeringcontrols.h"

#include "maverick/app_maverick_gen.h"

#include "app_maverick.h"

static THD_FUNCTION(maverick_drive_thread, arg);
static THD_WORKING_AREA(maverick_drive_thread_wa, 2048);

static THD_FUNCTION(maverick_steering_comms_thread, arg);
static THD_WORKING_AREA(maverick_steering_comms_thread_wa, 2048);

static THD_FUNCTION(maverick_steering_controls_thread, arg);
static THD_WORKING_AREA(maverick_steering_controls_thread_wa, 2048);

static THD_FUNCTION(maverick_drive_thread, arg){
    commands_printf("Maverick Drive");
    maverick_drive();
}

static THD_FUNCTION(maverick_steering_comms_thread, arg){
    commands_printf("Maverick Steering Comms");
    // maverick_steering_comms();
}

static THD_FUNCTION(maverick_steering_controls_thread, arg){
    commands_printf("Maverick Steering Controls");
    // maverick_steering_controls();
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
                chMtxObjectInit(&steering_pid_mtx);
                chMtxObjectInit(&steering_cmd_mtx);
                chMtxObjectInit(&steering_cmd_response_mtx);
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
}

// static double calculate_pid(double current_pos, double set_pos) {
//     static double integral = 0;
//     static double last_error = 0;

//     // Calculate Error
//     double error = set_pos - current_pos;

//     if (error < STEERING_DEADBAND && error > -STEERING_DEADBAND) error = 0;

//     // Calculate Integral
//     integral += error;
//     // Calculate Derivative
//     double derivative = error - last_error;

//     // Grab latest PID values
//     chMtxLock(&steering_pid_mtx);
//     steering_pid_values pid_v = steering_pid;
//     chMtxUnlock(&steering_pid_mtx);

//     // Calculate PID output
//     double out = pid_v.kp * error + pid_v.ki * integral + pid_v.kd * derivative;
//     // Ensure it doesn't go outside duty range
//     if (out > MAX_STEERING_DUTY) out = MAX_STEERING_DUTY;
//     else if (out < -MAX_STEERING_DUTY) out = -MAX_STEERING_DUTY;
//     else if (out < MIN_STEERING_DUTY && out > -MIN_STEERING_DUTY) out = 0;

//     last_error = error;
//     return out;
// }