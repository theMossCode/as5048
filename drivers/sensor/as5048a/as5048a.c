#include <drivers/sensor.h>
#include <logging/log.h>
#include "as5048a.h"

LOG_MODULE_REGISTER(AS5048A, CONFIG_SENSOR_LOG_LEVEL);

struct as5048a_data{
    uint16_t angle_raw;
    uint16_t magnitude_raw;
    uint16_t error_reg;
};

struct as5048a_config{
    struct spi_dt_spec spi;
	struct spi_cs_control cs;
    const struct as5048_bus_io *bus_io;
};

static inline int as5048a_bus_check(const struct device *dev)
{
    const struct as5048a_config *cfg = dev->config;
    return cfg->bus_io->check(&cfg->spi);
}

static inline int as5048a_reg_read(const struct device *dev,
				  uint16_t addr, uint16_t *buf)
{
	const struct as5048a_config *cfg = dev->config;
	return cfg->bus_io->read(&cfg->spi, addr, buf);
}

static inline int as5048a_reg_write(const struct device *dev, uint16_t reg,
				   uint16_t val)
{
	const struct as5048a_config *cfg = dev->config;
	return cfg->bus_io->write(&cfg->spi, reg, val);
}

static int as5048a_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
    const struct as5048a_data *data = (struct as5048a_data *)dev->data;

    int ret = 0;

    switch(chan){
        case SENSOR_CHAN_ROTATION:{
            ret = as5048a_reg_read(dev, AS5048_REG_ANGLE, (uint16_t*)&data->angle_raw);
            if(ret){
                return ret;
            }
            break;
        }
        default:{
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
        val->val1 = data->angle_raw;
    }
    else{
        LOG_DBG("Channel not supported\r\n");
        return -EINVAL;
    }
    return 0;
}

static int as5048a_init(const struct device *dev)
{
    struct as5048a_data *data = dev->data;

    int ret = 0;

    ret = as5048a_bus_check(dev);
    if(!ret){
        LOG_DBG("Bus check FAIL:\r\n");
        return ret;
    }

    ret = as5048a_reg_read(dev, AS5048_REG_CLEAR_ERR_FLAG, &data->error_reg);
    if(ret){
        LOG_DBG("err check FAIL:\r\n");
        return ret;
    }

    data->angle_raw = 0;
    data->magnitude_raw = 0;

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
    static const struct as5048a_config as5048a_config_##inst ={  	\
			.spi = SPI_INIT_DT(inst),								\
			.bus_io = &as5048_bus_io_spi							\
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


