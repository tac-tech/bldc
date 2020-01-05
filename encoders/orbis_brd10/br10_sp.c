#include <math.h>

#include <hal.h>
// #include <commands.h>

#include "soft_spi.h"
#include "br10_sp.h"

// #define BYTE2_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
// #define BYTE2_TO_BINARY(byte)  \
//   (byte & 0x8000 ? '1' : '0'), \
//   (byte & 0x4000 ? '1' : '0'), \
//   (byte & 0x2000 ? '1' : '0'), \
//   (byte & 0x1000 ? '1' : '0'), \
//   (byte & 0x800 ? '1' : '0'), \
//   (byte & 0x400 ? '1' : '0'), \
//   (byte & 0x200 ? '1' : '0'), \
//   (byte & 0x100 ? '1' : '0'), \
//   (byte & 0x80 ? '1' : '0'), \
//   (byte & 0x40 ? '1' : '0'), \
//   (byte & 0x20 ? '1' : '0'), \
//   (byte & 0x10 ? '1' : '0'), \
//   (byte & 0x08 ? '1' : '0'), \
//   (byte & 0x04 ? '1' : '0'), \
//   (byte & 0x02 ? '1' : '0'), \
//   (byte & 0x01 ? '1' : '0') 

// #define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
// #define BYTE_TO_BINARY(byte)  \
//   (byte & 0x80 ? '1' : '0'), \
//   (byte & 0x40 ? '1' : '0'), \
//   (byte & 0x20 ? '1' : '0'), \
//   (byte & 0x10 ? '1' : '0'), \
//   (byte & 0x08 ? '1' : '0'), \
//   (byte & 0x04 ? '1' : '0'), \
//   (byte & 0x02 ? '1' : '0'), \
//   (byte & 0x01 ? '1' : '0') 

gen_response br10_getGeneralResponse(){
    uint8_t send_byte[3] = {0x00, 0x00, 0x00};
    uint8_t receive_byte[3];

    spi_begin();
    spi_delay(100); // t_S in documentation => time between CS and first CLK rising edge
    spi_transfer(&receive_byte, &send_byte, 3);
    spi_end();
    spi_delay(100); // Pause time after CS is deactivated

    gen_response response;
    response.isError =   !(0b00000010 & receive_byte[1]);
    response.isWarning = !(0b00000001 & receive_byte[1]);

    response.position =  ((uint16_t)((receive_byte[0] << 6) | 
                                ((receive_byte[1] & 0b11111100) >> 2)))
                                * 360.0 / powf(2, 14);
    response.crc = receive_byte[2];
    return response;
}