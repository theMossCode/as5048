#include <drivers/sensor.h>
#include <logging/log.h>
#include "as5048a.h"

LOG_MODULE_REGISTER(AS5048A, CONFIG_SENSOR_LOG_LEVEL);

#define AS5048_RW_WRITE                    0
#define AS5048_RW_READ                     1

struct as5048a_data{
    uint16_t angle;
    uint16_t magnitude;
    uint16_t error;
};

struct as5048a_config{
    struct spi_dt_spec spi;
	struct spi_cs_control cs;
};

static inline uint8_t calculate_even_parity(uint16_t value)
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

static inline int as5048a_bus_check(const struct device *dev)
{
    const struct as5048a_config *cfg = dev->config;
    return spi_is_ready(&cfg->spi) ? 0 : -ENODEV;
}

static inline int as5048a_transceive(const struct device *dev, uint8_t rw, uint16_t cmd, uint16_t *rx_data)
{
	const struct as5048a_config *cfg = dev->config;
    uint8_t cmd_buf[2];
    uint8_t receive_buf[2];
    int ret = 0;

    cmd &= 0x3fff; // clear the parity and RW field
    cmd |= (rw << 14); // set read/write
    cmd |= ((uint16_t)calculate_even_parity(cmd) << 15); // set the parity bit

    cmd_buf[0] = (uint8_t)((cmd >> 8) & 0xff); // MSB
    cmd_buf[1] = (uint8_t)(cmd & 0xff); // LSB

	const struct spi_buf tx_buf = {
		.buf = cmd_buf,
		.len = sizeof(cmd_buf)
	};
	const struct spi_buf rx_buf = {
        .buf = receive_buf,
        .len = sizeof(receive_buf)
    };
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    if(ret < 0){
        LOG_DBG("SPI transceive FAIL: %d\r\n", ret);
        return ret;
    }

    *rx_data = receive_buf[0];// MSB
    *rx_data <<= 8;
    *rx_data |= receive_buf[1];// LSB

    return 0;   
}

static inline int as5048a_reg_read(const struct device *dev,
				  uint16_t addr, uint16_t *buf)
{
    uint16_t received_data;
    int ret = as5048a_transceive(dev, AS5048_RW_READ, addr, &received_data);
    if(ret < 0){
        LOG_DBG("reg read FAIL: %d\r\n", ret);
        return ret;  
    }

    ret = as5048a_transceive(dev, AS5048_RW_WRITE, AS5048_REG_NOP, &received_data);
    if(ret < 0){
        LOG_DBG("reg read FAIL %d\r\n", ret);
        return ret;         
    }

    if(AS5048_RECEIVED_DATA_ERR_CHECK(received_data)){
        LOG_DBG("err check FAIL: %4x\r\n", received_data);
        return -EBADMSG;
    }

    *buf = received_data & 0x3fff; // remove parity and error bits

    return 0;
}

static inline int as5048a_reg_write(const struct device *dev, uint16_t reg,
				   uint16_t val)
{;
    uint16_t received_data;
    int ret = as5048a_transceive(dev, AS5048_RW_WRITE, reg, &received_data);
    if(ret < 0){
        LOG_DBG("write FAIL: %d\r\n", ret);
        return ret;  
    }

    ret = as5048a_transceive(dev, AS5048_RW_WRITE, val, &received_data);
    if(ret < 0){
        LOG_DBG("write FAIL %d\r\n", ret);
        return ret;         
    }

    ret = as5048a_transceive(dev, AS5048_RW_WRITE, AS5048_REG_NOP, &received_data);
    if(ret < 0){
        LOG_DBG("Write confirm FAIL\r\n");
        return ret;
    }

    if(received_data != val){
        LOG_DBG("Write error\r\n Written: %u Read: %u", val, received_data);
        return -ENOMSG;
    }

    return 0;    
}

static int as5048a_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
    struct as5048a_data *data = (struct as5048a_data *)dev->data;
    int ret = 0;

    switch(chan){
        case SENSOR_CHAN_ALL: // FALL THROUGH
        case SENSOR_CHAN_ROTATION:{
            uint16_t received_data = 0;
            ret = as5048a_reg_read(dev, AS5048_REG_ANGLE, &received_data);
            if(ret < 0){
                LOG_DBG("Angle Fetch FAIL\r\n");
                return ret;
            }
            data->angle = received_data;
            break;
        }
        default:{
            LOG_WRN("Channel not supported\r\n");
            return -EINVAL;
        }
    }

    return 0;
}

static int as5048a_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val)
{
    struct as5048a_data *data = (struct as5048a_data *)dev->data;
    if(chan == SENSOR_CHAN_ROTATION){
        val->val1 = data->angle;
    }
    else{
        LOG_WRN("Channel not supported\r\n");
        return -EINVAL;
    }
    return 0;
}

static int as5048a_init(const struct device *dev)
{
    struct as5048a_data *data = dev->data;

    int ret = 0;

    ret = as5048a_bus_check(dev);
    if(ret<0){
        LOG_DBG("Bus check FAIL: %d\r\n", ret);
        return ret;
    }

    data->angle = 0;
    data->magnitude = 0;

    LOG_DBG("%s Init ok\r\n", dev->name);
    return 0;
}                                 

static const struct sensor_driver_api as5048a_api = {
    .sample_fetch = as5048a_sample_fetch,
    .channel_get = as5048a_channel_get
};

#define SPI_CFG_GET(inst)											\
	{																\
		.frequency = DT_INST_PROP_OR(inst, spi_max_frequency, 0U),	\
		.operation = AS5048_SPI_OPERATION,							\
		.slave = DT_INST_REG_ADDR(inst),							\
		.cs = SPI_CS_CONTROL_PTR_DT_INST(inst, 0)\
	}

#define SPI_INIT_DT(inst)							\
	{													\
		.bus = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(inst))),\
		.config = SPI_CFG_GET(inst)						\
	}

/* Main instantiation matcro */
#define AS5048_DEFINE(inst)                                     	\
    static struct as5048a_data as5048a_data_##inst;               	\
    static const struct as5048a_config as5048a_config_##inst = {  	\
			.spi = SPI_INIT_DT(inst),				                \
	};    															\
																	\
    DEVICE_DT_INST_DEFINE(inst,                                  	\
                        as5048a_init,                           	\
                        NULL,                                   	\
                        &as5048a_data_##inst,                     	\
                        &as5048a_config_##inst,                  	\
                        POST_KERNEL,                            	\
                        CONFIG_SENSOR_INIT_PRIORITY,             	\
                        &as5048a_api);                           
                        

/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(AS5048_DEFINE) 


