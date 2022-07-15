#ifndef AS5048_H
#define AS5048_H

#include <zephyr.h>
// #include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/spi.h>

#define DT_DRV_COMPAT   ams_as5048a

#define AS5048_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL)

#define AS5048_REG_NOP                  0x0000
#define AS5048_REG_CLEAR_ERR_FLAG       0x0001
#define AS5048_REG_PROG_CTRL            0x0003
#define AS5048_REG_OTP_ZERO_POS_HIGH    0x0016
#define AS5048_REG_OTP_ZERO_POS_LOW     0x0017
#define AS5048_REG_DIAG_AGC             0x3ffd
#define AS5048_REG_MAG                  0x3ffe
#define AS5048_REG_ANGLE                0x3fff

#define AS5048_RECEIVED_DATA_ERR_CHECK(val)        (val & (1 << 14))

#endif