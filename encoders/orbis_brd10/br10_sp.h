#ifndef ORBIS_BR10_SP_H_
#define ORBIS_BR10_SP_H_

#include <hal.h>

typedef struct {
    bool isError;
    bool isWarning;
    double position;
    char crc;
} gen_response;

typedef struct {
    gen_response gen_response;
    char serial_no[6];
} serial_no_response;

typedef struct {
    gen_response gen_response;
    float speed;
} speed_response;

typedef struct {
    gen_response gen_response;
    float temp;
} temp_response;

typedef struct {
    gen_response gen_response;
    bool isSignalAmplitudeHigh;
    bool isSignalAmplitudeLow;
    bool isTempOutOfRange;
    bool isSpeedHigh;
    bool isMultiturnError;
} detailed_status;

// Public Functions
gen_response br10_getGeneralResponse();
serial_no_response br10_getSerialNoResponse();
speed_response br10_getSpeedResponse();
temp_response br10_getTempResponse();
detailed_status br10_getDetailedStatusResponse();
void br10_setAsZeroPosition(double position, bool save);

// Private Functions
static bool check_crc(uint8_t *input_data, int length);
static gen_response compute_gen(uint8_t *received_bytes, int length);
static void spi_comms(uint8_t *receive_bytes, uint8_t *send_bytes, int length);
// TODO: Implement crc checking function

#endif // ORBIS_BR10_SP_H