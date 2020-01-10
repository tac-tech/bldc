#ifndef APP_MAVERICK_H_
#define APP_MAVERICK_H_

#include "soft_spi.h"

#define MAX_STEERING_DUTY   0.5
#define MIN_STEERING_DUTY   0.1

#define STEERING_DEADBAND   0.5

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

// Define some basic variables:
static uint8_t min_motor_id = 20; // minimum motor CAN/VESC id

// Function Definitions
void maverick_configure(app_configuration *conf);
void maverick_init(app_configuration *config);
void maverick_stop();

// Private Functions
static double calculate_pid(double current_pos, double set_pos);
static double map(double input, double in_min, double in_max, double out_min, double out_max);
#endif //APP_MAVERICK_H_