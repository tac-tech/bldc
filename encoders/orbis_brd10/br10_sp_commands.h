#ifndef ORBIS_BR10_SP_COMMANDS_H_
#define ORBIS_BR10_SP_COMMANDS_H_

#include "br10_sp.h"

typedef enum {
    GET_SERIAL_NO = 1,
    GET_SPEED,
    GET_TEMP,
    GET_DETAILED_STATUS
} steering_commands;

typedef struct {
    steering_commands cmd;
    double cmd_position;
} steering_cmd;

typedef struct {
    steering_commands cmd;
    // Position
    double current_position;
    // Status flags
    bool isError;
    bool isWarning;
    // Serial number
    char serial_no[6];
    // Speed
    float speed;
    // Temperature
    float temperature;
    // Detailed Status
    detailed_status detailed_status;

} steering_cmd_response;

#endif // ORBIS_BR10_SP_COMMANDS_H_