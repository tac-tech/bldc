#include <math.h>

#include <hal.h>
// #include <commands.h>

#include "soft_spi.h"
#include "br10_sp.h"

#define GEN_RESPONSE_LENGTH         3
#define SERIAL_NO_RESPONSE_LENGTH   (GEN_RESPONSE_LENGTH + 6)
#define SPEED_REQUEST_LENGTH        (GEN_RESPONSE_LENGTH + 2)
#define TEMP_REQUEST_LENGTH         (GEN_RESPONSE_LENGTH + 2)
#define DETAILED_STATUS_REQUEST     (GEN_RESPONSE_LENGTH + 1)

gen_response br10_getGeneralResponse(){
    uint8_t send_bytes[3] = {0};
    uint8_t receive_bytes[3];

    spi_comms(&receive_bytes, &send_bytes, 3);

    gen_response response;
    response.isError =   !(0b00000010 & receive_bytes[1]);
    response.isWarning = !(0b00000001 & receive_bytes[1]);

    response.position =  ((uint16_t)((receive_bytes[0] << 6) | 
                                ((receive_bytes[1] & 0b11111100) >> 2)))
                                * 360.0 / powf(2, 14);
    response.crc = receive_bytes[2];
    return response;
}



static void spi_comms(uint8_t *receive_bytes, uint8_t *send_bytes, int length){
    spi_begin();
    spi_delay(100); // t_S in documentation => time between CS and first CLK rising edge
    spi_transfer(receive_bytes, send_bytes, length);
    spi_end();
    spi_delay(100); // Pause time after CS is deactivated
}