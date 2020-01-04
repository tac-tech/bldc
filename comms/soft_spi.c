#include <hal.h>

#include "soft_spi.h"
#include "conf_general.h"

#ifndef SPI_SAMPLE_RATE_HZ
#define SPI_SAMPLE_RATE_HZ  200 000 // 200KHz
#endif // SPI_SAMPLE_RATE_HZ

#ifndef SPI_MISO_GPIO
// #define SPI_MISO_GPIO       GPIOB
#define SPI_MISO_GPIO       HW_SPI_PORT_MISO
#endif // SPI_MISO_GPIO

#ifndef SPI_MISO_PIN
// #define SPI_MISO_PIN        4
#define SPI_MISO_PIN        HW_SPI_PIN_MISO
#endif // SPI_MISO_PIN

#ifndef SPI_MOSI_GPIO
// #define SPI_MOSI_GPIO       GPIOB
#define SPI_MOSI_GPIO       HW_SPI_PORT_MOSI
#endif // SPI_MOSI_GPIO

#ifndef SPI_MOSI_PIN
// #define SPI_MOSI_PIN        5
#define SPI_MOSI_PIN        HW_SPI_PIN_MOSI
#endif // SPI_MOSI_PIN

#ifndef SPI_CLK_GPIO
// #define SPI_CLK_GPIO        GPIOB
#define SPI_CLK_GPIO        HW_SPI_PORT_SCK
#endif // SPI_MISO_GPIO

#ifndef SPI_CLK_PIN
// #define SPI_CLK_PIN         3
#define SPI_CLK_PIN         HW_SPI_PIN_SCK
#endif // SPI_MISO_PIN

// FIXME: Replace this with a proper system of CS pins
//          - Struct to hold the GPIO + Pin
//          - Array/Dictionary to hold multiple pins if necessary
#ifndef SPI_CS_GPIO
// #define SPI_CS_GPIO         GPIOA
#define SPI_CS_GPIO         HW_SPI_PORT_NSS
#endif // SPI_CS_GPIO

#ifndef SPI_CS_PIN
// #define SPI_CS_PIN          4
#define SPI_CS_PIN          HW_SPI_PIN_NSS
#endif // SPI_CS_PIN

// Redefine pal functions for more readable code
#define SET_SPI_CLK(x) palWritePad(SPI_CLK_GPIO, SPI_CLK_PIN, x)
#define SET_SPI_MOSI(x) palWritePad(SPI_MOSI_GPIO, SPI_MOSI_PIN, x)
#define SET_SPI_CS(x) palWritePad(SPI_CS_GPIO, SPI_CS_PIN, x)
#define READ_SPI_MISO() palReadPad(SPI_MISO_GPIO, SPI_MISO_PIN)

void spi_init(){
    // Set up pins
    palSetPadMode(SPI_MISO_GPIO, SPI_MISO_PIN, PAL_MODE_INPUT);
    palSetPadMode(SPI_MOSI_GPIO, SPI_MOSI_PIN, 
                    PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(SPI_CLK_GPIO, SPI_CLK_PIN, 
                    PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(SPI_CS_GPIO, SPI_CS_PIN,
                    PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
}

void spi_deinit(){
    palSetPadMode(SPI_MISO_GPIO, SPI_MISO_PIN, PAL_MODE_INPUT_PULLUP);
    palSetPadMode(SPI_MOSI_GPIO, SPI_MOSI_PIN, PAL_MODE_INPUT_PULLUP);
    palSetPadMode(SPI_CLK_GPIO, SPI_CLK_PIN, PAL_MODE_INPUT_PULLUP);
    palSetPadMode(SPI_CS_GPIO, SPI_CS_PIN, PAL_MODE_INPUT_PULLUP);
}

void spi_transfer(uint8_t *in_buffer, const uint8_t *out_buffer, int length){
    for (int i = 0; i < length; i++) {
        // uint8_t send_byte = out_buffer ? out_buffer[i] : 0x47;
		uint8_t send_byte = 0x47;
        uint8_t receive_byte = 0;
		systime_t time = chVTGetSystemTimeX();
		int read1, read2, read3;
        for (int bit = 0; bit < 8; bit++) {
            // Clock High
            SET_SPI_CLK(1);
            
			// Write MOSI bit and wait for switch
            SET_SPI_MOSI((send_byte >> (7-bit)) & 1);
            spi_delay(100);

			// Clock Low
            SET_SPI_CLK(0);

            // Read MISO bit
			read1 = READ_SPI_MISO();
			spi_delay(1);
			read2 = READ_SPI_MISO();
			spi_delay(1);
			read3 = READ_SPI_MISO();
			receive_byte <<= 1;
			if (utils_middle_of_3_int(read1, read2, read3)){
				receive_byte |= 1;
			}
			spi_delay(100);
        }
    }
}

void spi_transfer_old(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t receive = 0;

		for (int bit = 0;bit < 16;bit++) {
			//palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

			spi_delay(4);
			palSetPad(SPI_CLK_GPIO, SPI_CLK_PIN);
			spi_delay(4);
			
			int r1, r2, r3;
			r1 = palReadPad(SPI_MISO_GPIO, SPI_MISO_PIN);
			__NOP();
			r2 = palReadPad(SPI_MISO_GPIO, SPI_MISO_PIN);
			__NOP();
			r3 = palReadPad(SPI_MISO_GPIO, SPI_MISO_PIN);

			receive <<= 1;
			if (utils_middle_of_3_int(r1, r2, r3)) {
				receive |= 1;
			}

			palClearPad(SPI_CLK_GPIO, SPI_CLK_PIN);
			spi_delay(4);
		}

		if (in_buf) {
			in_buf[i] = receive;
		}
	}
}

void spi_begin() {
	palClearPad(SPI_CS_GPIO, SPI_CS_PIN); // Set to 0
}

void spi_end(void) {
	palSetPad(SPI_CS_GPIO, SPI_CS_PIN); // set to 1
}

static void spi_wait_until(systime_t time) {
	// FIXME: time may be wrapped around to 0, making system time always > than time
	// Can cause read/write errors in SPI. Need to deal with this by checking if close to end of uint32_t
	while (chVTGetSystemTimeX() < time){
		spi_delay(1);
	}
}

static void spi_delay(uint16_t length) {
	for (int i = 0; i < length; i++){
		__NOP();
	}
}