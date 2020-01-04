#include <math.h>

#include <hal.h>
// #include <commands.h>

#include "soft_spi.h"
#include "br10_sp.h"

gen_response br10_getGeneralResponse(){
    uint8_t send_byte[3] = {0x00, 0x00, 0x00};
    uint8_t receive_byte[3];

    spi_begin();
    spi_transfer(&receive_byte, &send_byte, 3);
    spi_end();
    
    // commands_printf("Byte 1: 0x%X", send_byte[0]);
    // commands_printf("Byte 2: 0x%X", send_byte[1]);
    // commands_printf("Byte 3: 0x%X", send_byte[2]);

    gen_response response;
    response.isError =   !(0b00000010 & receive_byte[1]);
    response.isWarning = !(0b00000001 & receive_byte[1]);

    response.position = (float) ((receive_byte[0] << 6) | (receive_byte[1] >> 2)); // *
                                //360.0 / powf(2, 14);

    response.crc = receive_byte[2];
    return response;
}