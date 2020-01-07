#ifndef ORBIS_BR10_SP_H_
#define ORBIS_BR10_SP_H_

#include <hal.h>

/**
 * @brief   General Response struct
 * @details Contains the following:
 *      - [bool] isError: Error flag
 *      - [bool] isWarning: Warning flag
 *      - [double] position: Encoder position
 *      - [char] crc: CRC from the encoder
 * @warning crc may be changed to a boolean
 */
typedef struct {
    bool isError;
    bool isWarning;
    double position;
    char crc;
} gen_response;

/**
 * @brief   Serial Number Response struct
 * @details Contains Serial Number + general response:
 *      - [gen_response] gen_response: General Response (see gen_response struct)
 *      - [char] serial_no: 6 character/byte serial number
 */
typedef struct {
    gen_response gen_response;
    char serial_no[6];
} serial_no_response;

/**
 * @brief   Speed response struct
 * @details Contains current speed + general response:
 *      - [gen_response] gen_response: General Response (see gen_response struct)
 *      - [float] speed: speed in RPM
 */
typedef struct {
    gen_response gen_response;
    float speed;
} speed_response;

/**
 * @brief   Temperature Response struct
 * @details Contains measured temperare + general response:
 *      - [gen_response] gen_response: General Response (see gen_response struct)
 *      - [float] temp: temperature in Celsius
 */
typedef struct {
    gen_response gen_response;
    float temp;
} temp_response;

/**
 * @brief   Detailed status struct
 * @details Contains flags showing detailed status of the encoder:
 *      - [gen_response] gen_response: General Response (see gen_response struct)
 *      - [bool] isSignalAmplitudeHigh: True if encoder is too close to magnet
 *      - [bool] isSignalAmplitudeLow: True if encoder is too far from magnet
 *      - [bool] isTempOutOfRange: True if temperature is outside operating range
 *      - [bool] isSpeedHigh: True if speed is too high for encoder to read
 *      - [bool] isMultiturnError: True if there is a multiturn error (if encoder has multiturn feature)
 */
typedef struct {
    gen_response gen_response;
    bool isSignalAmplitudeHigh;
    bool isSignalAmplitudeLow;
    bool isTempOutOfRange;
    bool isSpeedHigh;
    bool isMultiturnError;
} detailed_status;

////////////////////////////////////////////
// ---------- Public Functions ---------- //
////////////////////////////////////////////
/**
 * @brief   Get general response from encoder
 * @details Get the general response from the encoder. This includes:
 *              - Position (in degrees)
 *              - Warning Flag
 *              - Error Flag
 * @return  [gen_response] struct with interperated data
 */
gen_response br10_getGeneralResponse();

/**
 * @brief   Get Serial Number response from encoder
 * @details Get the general response + the serial number response from the encoder. This includes:
 *              - General Response (Position, Warning Flag, Error Flag)
 *              - 6 character Serial Number
 * @return  [serial_no_response] struct with interperated data
 */
serial_no_response br10_getSerialNoResponse();

/**
 * @brief   Get Speed response from encoder
 * @details Get the speed + general response from the encoder. This includes:
 *              - General Response (Position, Warning Flag, Error Flag)
 *              - Signed speed data in RPM (not validated)
 * @note    Speed data has not yet been validated
 *          TODO: Validate speed data with tacometer
 * @return  [speed_response] struct with interperated data
 */
//
speed_response br10_getSpeedResponse();

/**
 * @brief   Get Temperature response from encoder
 * @details Get the temperature + general response from the encoder. Useful when you get
 *          a high temperature warning and need to debug (see Detailed Status). This includes:
 *              - General Response (Position, Warning Flag, Error Flag)
 *              - Signed temperature (in degrees Celsius)
 * @return  [temp_response] struct with interperated data
 */
temp_response br10_getTempResponse();

/**
 * @brief   Get detailed status response from encoder
 * @details Get the detailed status report + general response from the encoder. Useful
 *          when debugging, when the LED light is orange or red, or when you get a warning
 *          or error flag in the general message. This includes:
 *              - General Response (Position, Warning Flag, Error Flag)
 *              - Signal amplitude is too high (encoder is too close to magnet)
 *              - Signal amplitude is too low (encoder is too far from magnet)
 *              - Temperature is out of range
 *              - Speed is too high
 *              - Multiturn error (only if using multiturn encoder)
 * @return  [detailed_status] struct with interperated data
 */
detailed_status br10_getDetailedStatusResponse();

/**
 * @brief   Set zero position of encoder
 * @details Sets zero position of encoder given the angle desired to be the new zero
 * @param   [double] Position from 0 to 360 (not including 360) in degrees that is desired
 *          to be the new zero position
 * @param   [bool] Save to non-volatile memory on encoder if True
 * @warning DO NOT USE! Encoder does not respond to these commands. This method was developed according
 *          to documentation. Currently in talks with company to debug issues
 */
void br10_setZeroPosition(double position, bool save);

///////////////////////////////////////////////
// ----------- Private Functions ----------- //
///////////////////////////////////////////////
/**
 * @brief   Check CRC for data integrity  
 * @details Computes CRC (cyclic redundancy check) of incoming data and compares it with
 *          CRC value outputed by the encoder
 * @warning CRC calculation is valid, but encoder is not outputing valid CRCs
 *          This function should NOT be used until it is verified
 * @param   [uint8_t *] Pointer to array where input data lies
 * @param   [int] length of the input_data array
 * @return  [bool] True if input_data is valid, false if it is not
 */
static bool check_crc(uint8_t *input_data, int length);

/**
 * @brief   Compute general response 
 * @details Compute general response (see genResponse struct) given input data
 *              Intererates the incoming bytes of data into human-readable data
 * @param   [uint8_t *] Pointer to array where the input data lies
 * @param   [int] length of the received_bytes array
 * @return  [gen_response] Returns struct with computed/interperated data
 */
static gen_response compute_gen(uint8_t *received_bytes, int length);

/**
 * @brief   SPI message system
 * @details Send and receive n bytes through SPI
 * 
 * @param   [uint8_t *] Pointer to array where the bytes received should be placed
 * @param   [uint8_t *] Pointer to array where the bytes to be sent are
 * @param   [int] length of the array above. Note both arrays should be the same length
 */
static void spi_comms(uint8_t *receive_bytes, uint8_t *send_bytes, int length);

#endif // ORBIS_BR10_SP_H