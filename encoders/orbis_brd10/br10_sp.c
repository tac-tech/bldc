#include <math.h>

#include <hal.h>
#include <commands.h>

#include "soft_spi.h"
#include "br10_sp.h"

#define GEN_RESPONSE_LENGTH             3
#define SERIAL_NO_RESPONSE_LENGTH       (GEN_RESPONSE_LENGTH + 6)
#define SPEED_RESPONSE_LENGTH           (GEN_RESPONSE_LENGTH + 2)
#define TEMP_RESPONSE_LENGTH            (GEN_RESPONSE_LENGTH + 2)
#define DETAILED_STATUS_RESPONSE_LENGTH (GEN_RESPONSE_LENGTH + 1)

gen_response br10_getGeneralResponse(){
    uint8_t send_bytes[GEN_RESPONSE_LENGTH] = {0};
    uint8_t receive_bytes[GEN_RESPONSE_LENGTH];

    spi_comms(&receive_bytes, &send_bytes, GEN_RESPONSE_LENGTH);

    return compute_gen(&receive_bytes, GEN_RESPONSE_LENGTH);
}

serial_no_response br10_getSerialNoResponse(){
    uint8_t send_bytes[SERIAL_NO_RESPONSE_LENGTH] = {0x76};
    return;
}

speed_response br10_getSpeedResponse(){
    uint8_t send_bytes[SPEED_RESPONSE_LENGTH] = {0x73};
    uint8_t received_bytes[SPEED_RESPONSE_LENGTH];

    spi_comms(&received_bytes, &send_bytes, SPEED_RESPONSE_LENGTH);

    speed_response response;

    response.gen_response = compute_gen(&received_bytes, SPEED_RESPONSE_LENGTH);
    bool isNegative = (received_bytes[2] & 0b10000000)  > 0 ? true : false;
    response.speed = ((((received_bytes[2] & 0b01111111) << 8) | 
                    (received_bytes[3] & 0xFF)) - 
                    ((isNegative) ? (pow(2, 15)) : 0)) / 10.0;

    return response;
}

temp_response br10_getTempResponse(){
    uint8_t send_bytes[TEMP_RESPONSE_LENGTH] = {0x74};
    uint8_t received_bytes[TEMP_RESPONSE_LENGTH];

    spi_comms(&received_bytes, &send_bytes, TEMP_RESPONSE_LENGTH);

    temp_response response;
    
    response.gen_response = compute_gen(&received_bytes, TEMP_RESPONSE_LENGTH);
    bool isNegative = (received_bytes[2] & 0b10000000)  > 0 ? true : false;
    response.temp = ((((received_bytes[2] & 0b01111111) << 8) | 
                    (received_bytes[3] & 0xFF)) - 
                    ((isNegative) ? (pow(2, 15)) : 0)) / 10.0;
    
    return response;
}

detailed_status br10_getDetailedStatusResponse(){
    uint8_t send_bytes[DETAILED_STATUS_RESPONSE_LENGTH] = {0x64};
    uint8_t receive_bytes[DETAILED_STATUS_RESPONSE_LENGTH];

    spi_comms(&receive_bytes, &send_bytes, DETAILED_STATUS_RESPONSE_LENGTH);

    detailed_status response;
    response.gen_response = compute_gen(&receive_bytes, DETAILED_STATUS_RESPONSE_LENGTH);
    
    response.isSignalAmplitudeHigh  = 0b10000000 & receive_bytes[2];
    response.isSignalAmplitudeLow   = 0b01000000 & receive_bytes[2];
    response.isTempOutOfRange       = 0b00100000 & receive_bytes[2];
    response.isSpeedHigh            = 0b00010000 & receive_bytes[2];
    response.isMultiturnError       = 0b00001000 & receive_bytes[2];

    return response;
}

static gen_response compute_gen(uint8_t *received_bytes, int length){
    gen_response response;
    response.isError =   !(0b00000010 & received_bytes[1]);
    response.isWarning = !(0b00000001 & received_bytes[1]);

    response.position =  ((double)((received_bytes[0] << 6) | 
                                ((received_bytes[1] & 0b11111100) >> 2)))
                                * 360.0 / powf(2, 14);
    response.crc = received_bytes[length - 1];
    return response;
}

static void spi_comms(uint8_t *receive_bytes, uint8_t *send_bytes, int length){
    spi_begin();
    spi_delay(100); // t_S in documentation => time between CS and first CLK rising edge
    spi_transfer(receive_bytes, send_bytes, length);
    spi_end();
    spi_delay(100); // Pause time after CS is deactivated
}