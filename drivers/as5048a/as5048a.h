#ifndef AS5048_H
#define AS5048_H

#include <zephyr.h>
// #include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/spi.h>

#define DT_DRV_COMPAT   ams_as5048a

#define AS5048_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

typedef int (*as5048_bus_check_fn)(const struct spi_dt_spec *bus);
typedef int (*as5048_reg_read_fn)(const struct spi_dt_spec *bus,
				  uint16_t addr, uint16_t *rx_data);
typedef int (*as5048_reg_write_fn)(const struct spi_dt_spec *bus,
				   uint16_t reg, uint16_t val);

struct as5048_bus_io {
	as5048_bus_check_fn check;
	as5048_reg_read_fn read;
	as5048_reg_write_fn write;
};

#define AS5048_SPI_OPERATION (SPI_WORD_SET(16) | SPI_TRANSFER_MSB |	\
			      SPI_MODE_CPOL | SPI_MODE_CPHA)
extern const struct as5048_bus_io as5048_bus_io_spi;

#define AS5048_REG_NOP                  0x0000
#define AS5048_REG_CLEAR_ERR_FLAG       0x0001
#define AS5048_REG_PROG_CTRL            0x0003
#define AS5048_REG_OTP_ZERO_POS_HIGH    0x0016
#define AS5048_REG_OTP_ZERO_POS_LOW     0x0017
#define AS5048_REG_DIAG_AGC             0x3ffd
#define AS5048_REG_MAG                  0x3ffe
#define AS5048_REG_ANGLE                0x3fff

#define AS5048_RX_ERR_CHECK(val)        (val & (1 << 14))

#endif