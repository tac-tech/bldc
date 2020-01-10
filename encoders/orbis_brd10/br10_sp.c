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

uint8_t crc_lookup_table[256] = {
0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C};


gen_response br10_getGeneralResponse(){
    uint8_t send_bytes[GEN_RESPONSE_LENGTH] = {0};
    uint8_t receive_bytes[GEN_RESPONSE_LENGTH];

    spi_comms(&receive_bytes, &send_bytes, GEN_RESPONSE_LENGTH);

    return compute_gen(&receive_bytes, GEN_RESPONSE_LENGTH);
}

serial_no_response br10_getSerialNoResponse(){
    uint8_t send_bytes[SERIAL_NO_RESPONSE_LENGTH] = {0x76};
    uint8_t received_bytes[SERIAL_NO_RESPONSE_LENGTH];

    spi_comms(&received_bytes, &send_bytes, SERIAL_NO_RESPONSE_LENGTH);

    serial_no_response response;
    
    response.gen_response = compute_gen(&received_bytes, SERIAL_NO_RESPONSE_LENGTH);

    for (int i = 0; i < sizeof(response.serial_no); i++){
        response.serial_no[i] = received_bytes[i + 2];
    }

    return response;
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

detailed_status_response br10_getDetailedStatusResponse(){
    uint8_t send_bytes[DETAILED_STATUS_RESPONSE_LENGTH] = {0x64};
    uint8_t receive_bytes[DETAILED_STATUS_RESPONSE_LENGTH];

    spi_comms(&receive_bytes, &send_bytes, DETAILED_STATUS_RESPONSE_LENGTH);

    detailed_status_response response;
    response.gen_response = compute_gen(&receive_bytes, DETAILED_STATUS_RESPONSE_LENGTH);
    
    response.status.isSignalAmplitudeHigh  = 0b10000000 & receive_bytes[2];
    response.status.isSignalAmplitudeLow   = 0b01000000 & receive_bytes[2];
    response.status.isTempOutOfRange       = 0b00100000 & receive_bytes[2];
    response.status.isSpeedHigh            = 0b00010000 & receive_bytes[2];
    response.status.isMultiturnError       = 0b00001000 & receive_bytes[2];

    return response;
}

void br10_setZeroPosition(double position, bool save){
    // This function is correctly implemented, and verified with a logic analyzer.
    // However, the encoder is not responding to this command. 
    uint8_t send_bytes[9] = {0xCD, 0xEF, 0x89, 0xAB, 0x5A};
    uint8_t receive_bytes[9];
    uint8_t save_command[5] = {0xCD, 0xEF, 0x89, 0xAB, 0x63};
    // uint8_t receive_bytes_save[5];

    if (position >= 360 || position < 0){
        commands_printf("position out of bounds");
        return;
    }

    uint16_t raw_pos = position * (pow(2, 14) - 1) / 360;

    send_bytes[7] = (uint8_t)((raw_pos & 0b1111111100000000) >> 8);
    send_bytes[8] = (uint8_t)(raw_pos & 0b0000000011111111);

    spi_comms(&receive_bytes, &send_bytes, 9);
    if (save){
        spi_comms(&receive_bytes, &save_command, 5);
    }
}

/* Calculate CRC from fixed length data buffer: uint8_t Buffer[numOfBytes] */
uint8_t CRC_Buffer(uint8_t numOfBytes, uint8_t *Buffer){   
    uint32_t  t;
    uint8_t   icrc;
    numOfBytes -= 1;   
    icrc        = 1;   
    t           = Buffer[0];   
    while (numOfBytes--)   {     
        t = Buffer[icrc++] ^ crc_lookup_table[t];   
    }   

    return crc_lookup_table[t];
}

static bool check_crc(uint8_t *input_data, int length){
    // Define variables needed
    uint32_t index;
    int len_nocrc = length - 1; // Length without the crc byte at the end

    // Calculate CRC
    index = input_data[0];
    for (int i = 1; i < length - 1; i++){
        index = input_data[i] ^ crc_lookup_table[index];
    }
    uint8_t calculated_crc = crc_lookup_table[index];

    return calculated_crc == ~input_data[length - 1];
}

static gen_response compute_gen(uint8_t *received_bytes, int length){
    gen_response response;
    response.isError =   !(0b00000010 & received_bytes[1]);
    response.isWarning = !(0b00000001 & received_bytes[1]);

    response.position =  ((double)((received_bytes[0] << 6) | 
                                ((received_bytes[1] & 0b11111100) >> 2)))
                                * 360.0 / powf(2, 14);
    response.crcCheck = check_crc(received_bytes, length);

    return response;
}

static void spi_comms(uint8_t *receive_bytes, uint8_t *send_bytes, int length){
    spi_begin();
    spi_delay(100); // t_S in documentation => time between CS and first CLK rising edge
    spi_transfer(receive_bytes, send_bytes, length);
    spi_end();
    spi_delay(100); // Pause time after CS is deactivated
}
