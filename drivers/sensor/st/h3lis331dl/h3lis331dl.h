/* ST Microelectronics H3LIS331DL IMU sensor driver
 *
 * Copyright (c) 2025 Helbling Technik AG
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/h3lis331dl.pdf
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_H3LIS331DL_H_
#define ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_H3LIS331DL_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <stmemsc.h>
#include "h3lis331dl_reg.h"

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_h3lis331dl, spi) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_h3lis331dl, spi)
#include <zephyr/drivers/spi.h>
#endif

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_h3lis331dl, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_h3lis331dl, i2c)
#include <zephyr/drivers/i2c.h>
#endif

#define H3LIS331DL_EN_BIT					0x01
#define H3LIS331DL_DIS_BIT					0x00

/* Accel sensor sensitivity grain is 61 ug/LSB */
#define GAIN_UNIT_XL				(61LL)

struct h3lis331dl_config {
	stmdev_ctx_t ctx;
	union {
#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_h3lis331dl, i2c)
		const struct i2c_dt_spec i2c;
#endif
#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_h3lis331dl, spi)
		const struct spi_dt_spec spi;
#endif
	} stmemsc_cfg;
	uint8_t accel_odr;
#define ACCEL_RANGE_DOUBLE	BIT(7)
#define ACCEL_RANGE_MASK	BIT_MASK(6)
	uint8_t accel_range;
#ifdef CONFIG_H3LIS331DL_TRIGGER
	const struct gpio_dt_spec gpio_drdy;
	uint8_t int_pin;
	bool trig_enabled;
#endif /* CONFIG_H3LIS331DL_TRIGGER */
};

// #define H3LIS331DL_SHUB_MAX_NUM_TARGETS			3

struct h3lis331dl_data {
	const struct device *dev;
	int16_t acc[3];
	uint32_t acc_gain;

	uint16_t accel_freq;
	uint8_t accel_fs;

#ifdef CONFIG_H3LIS331DL_TRIGGER
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t handler_drdy_acc;
	const struct sensor_trigger *trig_drdy_acc;
	sensor_trigger_handler_t handler_drdy_gyr;
	const struct sensor_trigger *trig_drdy_gyr;
	sensor_trigger_handler_t handler_drdy_temp;
	const struct sensor_trigger *trig_drdy_temp;

#if defined(CONFIG_H3LIS331DL_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_H3LIS331DL_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_H3LIS331DL_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_H3LIS331DL_TRIGGER */
};

#ifdef CONFIG_H3LIS331DL_TRIGGER
int h3lis331dl_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int h3lis331dl_init_interrupt(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_H3LIS331DL_H_ */
