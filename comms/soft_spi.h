#ifndef COMMS_SOFT_SPI_H_
#define COMMS_SOFT_SPI_H_

void spi_init();
void spi_deinit();
void spi_begin();
void spi_end();
void spi_transfer(uint8_t *in_buffer, const uint8_t *out_buffer, int length);
void spi_transfer_old(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_wait_until(systime_t time);
static void spi_delay(uint16_t length);

#endif // COMMS_SOFT_SPI_H