#ifndef APP_MAVERICK_GEN_H
#define APP_MAVERICK_GEN_H

// Shared system imports
#include <timeout.h>        // Used to reset timeout(watchdog)
#include <can.h>            // CAN interface
#include <commands.h>       // Basic commands, including print to console

// Shared custom imports
#include "encoders/orbis_brd10/br10_sp_commands.h"

#define MAX_STEERING_DUTY   0.5
#define MIN_STEERING_DUTY   0.1
#define STEERING_DEADBAND   0.5

///////////////////////////////////////////////
// ---------- Define Custom Types ---------- //
///////////////////////////////////////////////

typedef enum {
    NONE,
    DRIVE, 
    STEERING
} maverick_motor_type; 

typedef struct {
    float kp;
    float ki;
    float kd;
    float kd_filter;
    float angle_division;
} steering_pid_values;

///////////////////////////////////////////////////
// ---------- Define shared variables ---------- //
///////////////////////////////////////////////////

// Shared variables
static volatile app_configuration config;
static volatile maverick_motor_type motor_type;

static volatile bool drive_isRunning = false;
static volatile bool steering_comms_isRunning = false;
static volatile bool steering_controls_isRunning = false;
static volatile bool stop_now = true;

// Steering Thread mutexes + variables
static bool is_mtx_setup = false;

static mutex_t steering_cmd_mtx ;
static steering_cmd latest_steering_cmd;

static mutex_t steering_cmd_response_mtx;
static steering_cmd_response latest_steering_cmd_response;

static mutex_t steering_pid_mtx;
static steering_pid_values steering_pid;


// Define some basic variables:
static uint8_t min_motor_id = 20; // minimum motor CAN/VESC id

////////////////////////////////////////////
// ---------- Shared functions ---------- //
////////////////////////////////////////////
double calculate_pid(steering_pid_values pid, double current_pos, double set_pos);
double map(double input, double in_min, double in_max, double out_min, double out_max);

#endif // APP_MAVERICK_GEN_H