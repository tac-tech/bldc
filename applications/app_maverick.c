// Include system files
#include <ch.h>             // ChibiOS general file
#include <hal.h>            // ChibiOS hardware abstraction layer
#include <mc_interface.h>   // Motor Controller interface
#include <hw.h>             // Pin mapping on the hardware
#include <timeout.h>        // Used to reset timeout
#include <can.h>            // CAN interface
#include <commands.h>       // Basic commands, including print to console

#include "encoders/orbis_brd10/br10_sp.h"
#include "encoders/orbis_brd10/br10_sp_commands.h"
#include "app_maverick.h"

// Private variables
static volatile app_configuration config;
static volatile maverick_motor_type motor_type;

static volatile bool drive_isRunning = false;
static volatile bool steering_comms_isRunning = false;
static volatile bool steering_controls_isRunning = false;
static volatile bool stop_now = true;

// Steering Thread mutexes + variables
static bool is_mtx_setup = false;

static mutex_t steering_pos_mtx;
static double steering_pos = -1.0;

static mutex_t steering_cmd_mtx ;
static steering_cmd latest_steering_cmd;

static mutex_t steering_cmd_response_mtx;
static steering_cmd_response latest_steering_cmd_response;

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
        chThdSleepMilliseconds(1000);
        timeout_reset();
    }
    commands_printf("Drive thread finished!");
    drive_isRunning = false;
}

static THD_FUNCTION(maverick_steering_comms_thread, arg){
    (void)arg;
    chRegSetThreadName("MAVERICK_STEERING_COMMS");
    steering_comms_isRunning = true;
    steering_commands last_cmd = NONE;
    steering_cmd next_cmd;
    steering_cmd_response last_response;
    while (!stop_now){
        if (last_cmd == NONE){
            next_cmd.cmd = GET_DETAILED_STATUS;
            next_cmd.cmd_position = 5;
        } else {
            next_cmd.cmd = NONE;
            next_cmd.cmd_position = 10;
        }
        chMtxLock(&steering_cmd_mtx);
        latest_steering_cmd = next_cmd;
        chMtxUnlock(&steering_cmd_mtx);

        chMtxLock(&steering_cmd_response_mtx);
        last_response = latest_steering_cmd_response;
        commands_printf("Position: %f", last_response.current_position);
        chMtxUnlock(&steering_cmd_response_mtx);
        
        last_cmd = next_cmd.cmd;
        chThdSleepMilliseconds(5000);
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

    // Define all variables to be used in the loop
    steering_cmd latest_cmd;

    serial_no_response enc_serial_no_response;
    speed_response enc_speed_response;
    temp_response enc_temp_response;
    detailed_status_response enc_detailed_status_response;
    gen_response enc_gen_response;

    while(!stop_now) {
        // Read most current command
        if (chMtxTryLock(&steering_cmd_mtx)){
            latest_cmd = latest_steering_cmd;
            chMtxUnlock(&steering_cmd_mtx);
        } else {
            // TODO: Implement a mutex skip counter to ensure latest command isn't getting skipped every time
        }

        // Get the current encoder value
        steering_cmd_response response;
        response.cmd = latest_cmd.cmd;
        switch(response.cmd){
            case GET_SERIAL_NO:;
                enc_serial_no_response = br10_getSerialNoResponse();
                response.current_position = enc_serial_no_response.gen_response.position;
                response.isError = enc_serial_no_response.gen_response.isError;
                response.isWarning = enc_serial_no_response.gen_response.isWarning;
                // response.serial_no = enc_response.serial_no;
                break;
            case GET_SPEED:;
                enc_speed_response = br10_getSpeedResponse();
                response.current_position = enc_speed_response.gen_response.position;
                response.isError = enc_speed_response.gen_response.isError;
                response.isWarning = enc_speed_response.gen_response.isWarning;
                response.speed = enc_speed_response.speed;
                break;
            case GET_TEMP:;
                enc_temp_response = br10_getTempResponse();
                response.current_position = enc_temp_response.gen_response.position;
                response.isError = enc_temp_response.gen_response.isError;
                response.isWarning = enc_temp_response.gen_response.isWarning;
                response.temperature = enc_temp_response.temp;
                break;
            case GET_DETAILED_STATUS:;
                enc_detailed_status_response = br10_getDetailedStatusResponse();
                response.current_position = enc_detailed_status_response.gen_response.position;
                response.isError = enc_detailed_status_response.gen_response.isError;
                response.isWarning = enc_detailed_status_response.gen_response.isWarning;
                response.detailed_status = enc_detailed_status_response.status;
                break;
            default:;
                enc_gen_response = br10_getGeneralResponse();
                response.current_position = enc_gen_response.position;
                response.isError = enc_gen_response.isError;
                response.isWarning = enc_gen_response.isWarning;
                break;
        }
        // TODO: Add CRC check
        
        // Calculate PID

        // Set motor
        if (latest_cmd.cmd == NONE){
            // mc_interface_set_pid_speed(2000);
            mc_interface_set_duty(0.2);
        } else {
            // mc_interface_set_pid_speed(2500);
            mc_interface_set_duty(0.25);
        }
        // Return command response
        chMtxLock(&steering_cmd_response_mtx);
        latest_steering_cmd_response = response;
        chMtxUnlock(&steering_cmd_response_mtx);
        // TODO: Implement a mailbox system
        // int return_cntr = 0;
        // while (!chMtxTryLock(&steering_cmd_response_mtx)){
        //     if (return_cntr > 5){
        //         commands_printf("Maverick Steering Controls Thread: "
        //             "Error trying to unlock command response mutex!")
        //     }            
        //     return_cntr++;
        //     chThdSleepMilliseconds(1);
        // }

        // Thread sleep
        // TODO: replace with sleep until to better control cycle speed
        chThdSleepMilliseconds(100);
        timeout_reset();
    }
    spi_deinit();
    chMtxUnlockAll();
    steering_controls_isRunning = false;
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
    // &steering_pos_mtx = NULL;
}