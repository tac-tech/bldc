// Include system files
#include <ch.h>             // ChibiOS general file
#include <hal.h>            // ChibiOS hardware abstraction layer
#include <mc_interface.h>   // Motor Controller interface
#include <hw.h>             // Pin mapping on the hardware
#include <timeout.h>        // Used to reset timeout
#include <can.h>            // CAN interface
#include <commands.h>       // Basic commands, including print to console

#include "app_maverick.h"

// Private variables
static volatile app_configuration config;
static volatile maverick_motor_type motor_type;


void maverick_configure(app_configuration *conf){
    config = *conf;
    commands_printf("%d", config.controller_id);
    if (config.controller_id > min_motor_id){             // 
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
            break;
        case STEERING:
            commands_printf("Steering");
            break;
        default:
            commands_printf("Default");
            break;
    }
}

void maverick_stop(){
    // TODO: Implement
    motor_type = NONE;
    return;
}