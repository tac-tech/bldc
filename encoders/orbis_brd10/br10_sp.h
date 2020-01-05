#ifndef ORBIS_BR10_SP_H_
#define ORBIS_BR10_SP_H_

#include <hal.h>

typedef struct {
    bool isError;
    bool isWarning;
    double position;
    char crc;
} gen_response;

struct serial_no_response {
    gen_response gen_response;
    char serial_no[6];
};

struct speed_request {
    gen_response gen_response;
    float speed;
};

struct temp_request {
    gen_response gen_response;
    float temp;
};

struct detailed_status {
    gen_response gen_response;
    bool isSignalAmplitudeHigh;
    bool isSignalAmplitudeLow;
    bool isTempOutOfRange;
    bool isSpeedHigh;
    bool isMultiturnError;
};

gen_response br10_getGeneralResponse();

#endif // ORBIS_BR10_SP_H