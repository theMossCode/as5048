/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <sys/printk.h>


/*
 * Get a device structure from a devicetree node with compatible
 * "ams,as5048a". (If there are multiple, just pick one.)
 */
static const struct device *get_as5048a_device(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(ams_as5048a);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

void main(void)
{
	printk("Init device\r\n");
	const struct device *dev = get_as5048a_device();

	if (dev == NULL) {
		return;
	}

	while (1) {
		struct sensor_value rotation;
		sensor_sample_fetch(dev);
		if(sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &rotation) == 0){
			printk("Angle: %i\r\n", rotation.val1);
		}

		k_sleep(K_MSEC(1000));
	}
}
