#ifndef APP_MAVERICK_H_
#define APP_MAVERICK_H_

typedef enum {
    NONE,
    DRIVE, 
    STEERING
} maverick_motor_type; 

// Define some basic variables:
static uint8_t min_motor_id = 20; // minimum motor CAN/VESC id

// Define Threads

// Function Definitions
void maverick_configure(app_configuration *conf);
void maverick_init(app_configuration *config);
void maverick_stop();

#endif //APP_MAVERICK_H_