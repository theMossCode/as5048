#include <logging/log.h>
#include "as5048a.h"

#define AS5048_WRITE                    0
#define AS5048_READ                     1

LOG_MODULE_DECLARE(AS5048A, CONFIG_SENSOR_LOG_LEVEL);

#define AS5048_RX_ERR_CHECK(val)        (val & (1 << 14))

static uint8_t calculate_even_parity(uint16_t value)
{
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;    
}

static int as5048_transfer(const struct spi_dt_spec *bus, uint8_t op, uint16_t tx_val, uint16_t *rx_val)
{
    uint16_t addr_val = tx_val & 0x3fff;
    addr_val |= (op << 14); 
    addr_val |= (calculate_even_parity(addr_val) << 15);
	uint8_t tx_data[2] = {(uint8_t)(addr_val & 0xff), (uint8_t)((addr_val >> 8) & 0x3f)};
    uint8_t rx_data[2];
	const struct spi_buf tx_buf = {
		.buf = tx_data,
		.len = 2
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = 2
    };
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	int ret = spi_transceive_dt(bus, &tx, &rx);
	if (ret) {
		LOG_DBG("spi_transceive FAIL %d\r\n", ret);
		return ret;
	}

    *rx_val = rx_data[0];
    *rx_val <<= 8;
    *rx_val |= rx_data[1];

	return 0;    
}

static int as5048_bus_check_spi(const struct spi_dt_spec *bus)
{
	return spi_is_ready(bus) ? 0 : -ENODEV;
}

static int as5048_reg_read_spi(const struct spi_dt_spec *bus,
			       uint16_t addr, uint16_t *rx_data)
{
    uint16_t rx_val;
    int ret = as5048_transfer(bus, addr, AS5048_READ, &rx_val);
    if(ret){
        LOG_DBG("reg read FAIL %d\r\n", ret);
        return ret;  
    }

    ret = as5048_transfer(bus, AS5048_REG_NOP, AS5048_WRITE, &rx_val);
    if(ret){
        LOG_DBG("reg read FAIL %d\r\n", ret);
        return ret;         
    }

    if(AS5048_RX_ERR_CHECK(rx_val)){
        LOG_DBG("err check FAIL\r\n");
        return -EPROTO;
    }

    *rx_data = rx_val & 0x3fff;

    return 0;
}

static int as5048_reg_write_spi(const struct spi_dt_spec *bus,
				uint16_t reg, uint16_t val)
{
    int ret = as5048_transfer(bus, AS5048_WRITE, reg, NULL);
    if(ret){
        LOG_DBG("write reg FAIL %d\r\n", ret);
        return ret;
    }

    ret = as5048_transfer(bus, AS5048_WRITE, val, NULL);
    if(ret){
        LOG_DBG("write val FAIL %d\r\n", ret);
        return ret;
    }

    // confirm successful write
    uint16_t rx_data = 0xffff;
    ret = as5048_transfer(bus, AS5048_WRITE, AS5048_REG_NOP, &rx_data);
    if(ret){
        LOG_DBG("write confirm FAIL %d\r\n", ret);
        return ret;
    }

    if(rx_data != val){
        LOG_DBG("write confirm FAIL %d\r\n", -ENODATA);
        return -EPROTO;
    }

	return 0;
}

const struct as5048_bus_io as5048_bus_io_spi = {
	.check = as5048_bus_check_spi,
	.read = as5048_reg_read_spi,
	.write = as5048_reg_write_spi,
};