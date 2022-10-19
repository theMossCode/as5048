#include <drivers/sensor.h>
#include <logging/log.h>
#include "as5048a.h"

LOG_MODULE_REGISTER(AS5048A, CONFIG_SENSOR_LOG_LEVEL);

#define AS5048_RECEIVED_DATA_ERR_CHECK_U8(val)        (val & (1 << 7))
#define AS5048_RW_READ                     0x4000

#define SENSOR_DATA_MULTIPLIER              1000000

struct hall_data{
    uint8_t mag[2];
    uint16_t agc;
    uint8_t ang[2];
};

uint8_t m_tx_buffer[2];
uint8_t m_rx_buffer[2];

struct as5048a_config{
    struct spi_dt_spec spi;
	struct spi_cs_control cs;
};

static inline uint8_t calculate_even_parity(uint16_t value)
{
	uint8_t cnt = 0;
	for (uint8_t i = 0; i < 16; i++){
		if (value & 0x1){
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

static inline int as5048a_transceive(const struct device *dev, uint16_t cmd)
{
	const struct as5048a_config *cfg = dev->config;
    int ret = 0;

    cmd |= ((uint16_t)calculate_even_parity(cmd) << 15); // set the parity bit

    m_tx_buffer[0] = (uint8_t)((cmd >> 8) & 0xff); // MSB
    m_tx_buffer[1] = (uint8_t)(cmd & 0xff); // LSB

	const struct spi_buf tx_buf = {
		.buf = m_tx_buffer,
		.len = sizeof(m_tx_buffer)
	};
	const struct spi_buf rx_buf = {
        .buf = m_rx_buffer,
        .len = sizeof(m_rx_buffer)
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

    return 0;   
}

static inline int as5048a_reg_read(const struct device *dev, uint16_t addr)
{
    int ret = as5048a_transceive(dev, addr | AS5048_RW_READ);
    if(ret < 0){
        LOG_DBG("reg read FAIL: %d\r\n", ret);
        return ret;  
    }

    ret = as5048a_transceive(dev, AS5048_REG_NOP);
    if(ret < 0){
        LOG_DBG("reg read FAIL %d\r\n", ret);
        return ret;         
    }

    // Check errors
    if(AS5048_RECEIVED_DATA_ERR_CHECK_U8(m_rx_buffer[0])){
        LOG_DBG("err check FAIL: MSB=%2x\r\n", m_rx_buffer[0]);
        // clear error
        ret = as5048a_transceive(dev, AS5048_REG_CLEAR_ERR_FLAG | AS5048_RW_READ);
        if(ret){
            LOG_DBG("clear errors FAIL\r\n");
            return ret;
        }
        return -EBADMSG;
    }

    // check parity
    uint16_t data_u16 = (((uint16_t)m_rx_buffer[0] << 8) | m_rx_buffer[1]) & 0x7fff;
    uint8_t parity_bit = m_rx_buffer[0] & 0x80;
    if(((calculate_even_parity(data_u16) >> 8) & 0x80) != parity_bit){
        LOG_DBG("parity check FAIL: MSB=%2x\r\n", m_rx_buffer[0]);
        return -EBADMSG;
    }

    m_rx_buffer[0] &= 0x3f;

    return 0;
}

static inline int as5048a_reg_write(const struct device *dev, uint16_t reg,
				   uint16_t val)
{;
    int ret = as5048a_transceive(dev, reg);
    if(ret < 0){
        LOG_DBG("write FAIL: %d\r\n", ret);
        return ret;  
    }

    ret = as5048a_transceive(dev, val);
    if(ret < 0){
        LOG_DBG("write FAIL %d\r\n", ret);
        return ret;         
    }

    return 0;    
}

static int as5048a_fetch_all(const struct device *dev)
{
    struct hall_data *h_data = (struct hall_data *)dev->data;
    int ret = 0;
    for(int i=0; i<3; ++i){
        if(i == 0){
            ret = as5048a_reg_read(dev, AS5048_REG_DIAG_AGC);
            if(ret){
                LOG_DBG("AGC fetch FAIL\r\n");
                return ret;
            }           
            h_data->agc = (uint16_t)m_rx_buffer[0] << 8;
            h_data->agc |= m_rx_buffer[1];
        }
        else if(i == 1){
            ret = as5048a_reg_read(dev, AS5048_REG_MAG);
            if(ret){
                LOG_DBG("MAG fetch FAIL\r\n");
                return ret;
            }           
            h_data->mag[0] = m_rx_buffer[0];
            h_data->mag[1] = m_rx_buffer[1];            
        }
        else if(i == 2){
            ret = as5048a_reg_read(dev, AS5048_REG_ANGLE);
            if(ret){
                LOG_DBG("ANGLE fetch FAIL\r\n");
                return ret;
            }           
            h_data->ang[0] = m_rx_buffer[0];
            h_data->ang[1] = m_rx_buffer[1];
        }
    }
    
    return 0;
}

static int as5048a_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
    struct hall_data *data= (struct hall_data *)dev->data;
    int ret = 0;

    switch(chan){
        case SENSOR_CHAN_ALL:{
            ret = as5048a_fetch_all(dev);
            if(ret){
                return ret;
            }
            break;
        }
        case SENSOR_CHAN_ROTATION:{
            ret = as5048a_reg_read(dev, AS5048_REG_ANGLE);
            if(ret < 0){
                LOG_DBG("Angle Fetch FAIL\r\n");
                return ret;
            }
            data->ang[0] = m_rx_buffer[0];
            data->ang[1] = m_rx_buffer[1];
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
    struct hall_data *data = (struct hall_data *)dev->data;
    if(chan == SENSOR_CHAN_ROTATION){
        uint16_t angle_raw = ((uint16_t)data->ang[0] << 8) | (uint16_t)data->ang[1];
        uint32_t angle = ((double)angle_raw * (360.0/16384.0)) * (SENSOR_DATA_MULTIPLIER);
        int32_t val1 = (int32_t)(angle / (SENSOR_DATA_MULTIPLIER));
        int32_t val2 = (int32_t)(angle % (SENSOR_DATA_MULTIPLIER));
        val->val1 = val1;
        val->val2 = val2;
    }
    else{
        LOG_WRN("Channel not supported\r\n");
        return -EINVAL;
    }
    return 0;
}

static int as5048a_init(const struct device *dev)
{
    struct hall_data *data = dev->data;
    int ret = 0;

    ret = as5048a_bus_check(dev);
    if(ret<0){
        LOG_DBG("Bus check FAIL: %d\r\n", ret);
        return ret;
    }

    memset(data, 0x00, sizeof(struct hall_data));

    LOG_DBG("%s Init ok\r\n", dev->name);
    return 0;
}                                 

static const struct sensor_driver_api as5048a_api = {
    .sample_fetch = as5048a_sample_fetch,
    .channel_get = as5048a_channel_get
};

#define SPI_CFG_GET(inst)											\
	{																\
		.frequency = DT_INST_PROP_OR(inst, spi_max_frequency, 1000000U),	\
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
    static struct hall_data hall_data_##inst;               	\
    static const struct as5048a_config as5048a_config_##inst = {  	\
			.spi = SPI_INIT_DT(inst),				                \
	};    															\
																	\
    DEVICE_DT_INST_DEFINE(inst,                                  	\
                        as5048a_init,                           	\
                        NULL,                                   	\
                        &hall_data_##inst,                     	\
                        &as5048a_config_##inst,                  	\
                        POST_KERNEL,                            	\
                        CONFIG_SENSOR_INIT_PRIORITY,             	\
                        &as5048a_api);                           
                        

/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(AS5048_DEFINE) 


