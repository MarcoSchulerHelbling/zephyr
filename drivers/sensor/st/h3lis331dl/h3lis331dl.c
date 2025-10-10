/* ST Microelectronics H3LIS331DL IMU sensor driver
 *
 * Copyright (c) 2025 Helbling Technik AG
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/h3lis331dl.pdf
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "h3lis331dl.h"
#include "h3lis331dl_reg.h"

LOG_MODULE_REGISTER(H3LIS331DL, CONFIG_SENSOR_LOG_LEVEL);

#include <stdint.h>

static const uint16_t h3lis331dl_dr_map[] = {50, 100, 400, 1000};

static int h3lis331dl_freq_to_odr_val(uint16_t freq)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(h3lis331dl_dr_map); i++) {
		if (freq <= h3lis331dl_dr_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int h3lis331dl_odr_to_freq_val(uint16_t odr)
{
	/* for valid index, return value from map */
	if (odr < ARRAY_SIZE(h3lis331dl_dr_map)) {
		return h3lis331dl_dr_map[odr];
	}

	/* invalid index, return last entry */
	return h3lis331dl_dr_map[ARRAY_SIZE(h3lis331dl_dr_map) - 1];
}

/* Accel sensor sensitivity grain is 49 mg/LSB */
#define SENSI_GRAIN_ACCEL 49LL

static const uint16_t h3lis331dl_accel_fs_map[] = {100, 200, 400};
static const uint16_t h3lis331dl_accel_fs_sens[] = {1, 2, 4};

static int h3lis331dl_accel_range_to_fs_val(int32_t range, bool double_range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(h3lis331dl_accel_fs_map); i++) {
		if (range == (h3lis331dl_accel_fs_map[i] << double_range)) {
			return i;
		}
	}

	return -EINVAL;
}

static int h3lis331dl_accel_fs_val_to_gain(int fs, bool double_range)
{
	/* Range of ±2G has a LSB of GAIN_UNIT_XL, thus divide by 2 */
	return double_range ?
		h3lis331dl_accel_fs_map[fs] * GAIN_UNIT_XL :
		h3lis331dl_accel_fs_map[fs] * GAIN_UNIT_XL / 2;
}

static int h3lis331dl_accel_set_fs_raw(const struct device *dev, uint8_t fs)
{
	const struct h3lis331dl_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct h3lis331dl_data *data = dev->data;

	if (h3lis331dl_xl_full_scale_set(ctx, fs) < 0) {
		return -EIO;
	}

	data->accel_fs = fs;

	return 0;
}

// static int h3lis331dl_accel_set_odr_raw(const struct device *dev, uint8_t odr)
// {
// 	const struct h3lis331dl_config *cfg = dev->config;
// 	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
// 	struct h3lis331dl_data *data = dev->data;

// 	if (h3lis331dl_xl_data_rate_set(ctx, odr) < 0) {
// 		return -EIO;
// 	}

// 	data->accel_freq = h3lis331dl_odr_to_freq_val(odr);

// 	return 0;
// }

// static int h3lis331dl_accel_odr_set(const struct device *dev, uint16_t freq)
// {
// 	int odr;

// 	odr = h3lis331dl_freq_to_odr_val(freq);
// 	if (odr < 0) {
// 		return odr;
// 	}

// 	if (h3lis331dl_accel_set_odr_raw(dev, odr) < 0) {
// 		LOG_DBG("failed to set accelerometer sampling rate");
// 		return -EIO;
// 	}

// 	return 0;
// }

// static int h3lis331dl_accel_range_set(const struct device *dev, int32_t range)
// {
// 	int fs;
// 	struct h3lis331dl_data *data = dev->data;
// 	const struct h3lis331dl_config *cfg = dev->config;
// 	bool range_double = !!(cfg->accel_range & ACCEL_RANGE_DOUBLE);

// 	fs = h3lis331dl_accel_range_to_fs_val(range, range_double);
// 	if (fs < 0) {
// 		return fs;
// 	}

// 	if (h3lis331dl_accel_set_fs_raw(dev, fs) < 0) {
// 		LOG_DBG("failed to set accelerometer full-scale");
// 		return -EIO;
// 	}

// 	data->acc_gain = h3lis331dl_accel_fs_val_to_gain(fs, range_double);
// 	return 0;
// }

// static int h3lis331dl_accel_config(const struct device *dev,
// 				enum sensor_channel chan,
// 				enum sensor_attribute attr,
// 				const struct sensor_value *val)
// {
// 	switch (attr) {
// 	case SENSOR_ATTR_FULL_SCALE:
// 		return h3lis331dl_accel_range_set(dev, sensor_ms2_to_g(val));
// 	case SENSOR_ATTR_SAMPLING_FREQUENCY:
// 		return h3lis331dl_accel_odr_set(dev, val->val1);
// 	default:
// 		LOG_DBG("Accel attribute not supported.");
// 		return -ENOTSUP;
// 	}

// 	return 0;
// }

// static int h3lis331dl_gyro_odr_set(const struct device *dev, uint16_t freq)
// {
// 	int odr;

// 	odr = h3lis331dl_freq_to_odr_val(freq);
// 	if (odr < 0) {
// 		return odr;
// 	}

// 	if (h3lis331dl_gyro_set_odr_raw(dev, odr) < 0) {
// 		LOG_DBG("failed to set gyroscope sampling rate");
// 		return -EIO;
// 	}

// 	return 0;
// }

static int h3lis331dl_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
// 	switch (chan) {
// 	case SENSOR_CHAN_ACCEL_XYZ:
// 		return h3lis331dl_accel_config(dev, chan, attr, val);
// 	case SENSOR_CHAN_GYRO_XYZ:
// 		return h3lis331dl_gyro_config(dev, chan, attr, val);
// 	default:
// 		LOG_WRN("attr_set() not supported on this channel.");
// 		return -ENOTSUP;
// 	}

	return 0;
}

// static int h3lis331dl_sample_fetch_accel(const struct device *dev)
// {
// 	const struct h3lis331dl_config *cfg = dev->config;
// 	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
// 	struct h3lis331dl_data *data = dev->data;

// 	if (h3lis331dl_acceleration_raw_get(ctx, data->acc) < 0) {
// 		LOG_DBG("Failed to read sample");
// 		return -EIO;
// 	}

// 	return 0;
// }

static int h3lis331dl_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
// 	switch (chan) {
// 	case SENSOR_CHAN_ACCEL_XYZ:
// 		h3lis331dl_sample_fetch_accel(dev);
// 		break;
// 	case SENSOR_CHAN_GYRO_XYZ:
// 		h3lis331dl_sample_fetch_gyro(dev);
// 		break;
// #if defined(CONFIG_H3LIS331DL_ENABLE_TEMP)
// 	case SENSOR_CHAN_DIE_TEMP:
// 		h3lis331dl_sample_fetch_temp(dev);
// 		break;
// #endif
// 	case SENSOR_CHAN_ALL:
// 		h3lis331dl_sample_fetch_accel(dev);
// 		h3lis331dl_sample_fetch_gyro(dev);
// #if defined(CONFIG_H3LIS331DL_ENABLE_TEMP)
// 		h3lis331dl_sample_fetch_temp(dev);
// #endif
// #if defined(CONFIG_H3LIS331DL_SENSORHUB)
// 		if (data->shub_inited) {
// 			h3lis331dl_sample_fetch_shub(dev);
// 		}
// #endif
// 		break;
// 	default:
// 		return -ENOTSUP;
// 	}

	return 0;
}

// static inline void h3lis331dl_accel_convert(struct sensor_value *val, int raw_val,
// 					 uint32_t sensitivity)
// {
// 	int64_t dval;

// 	/* Sensitivity is exposed in ug/LSB */
// 	/* Convert to m/s^2 */
// 	dval = (int64_t)(raw_val) * sensitivity;
// 	sensor_ug_to_ms2(dval, val);
// }

// static inline int h3lis331dl_accel_get_channel(enum sensor_channel chan,
// 					    struct sensor_value *val,
// 					    struct h3lis331dl_data *data,
// 					    uint32_t sensitivity)
// {
// 	uint8_t i;

// 	switch (chan) {
// 	case SENSOR_CHAN_ACCEL_X:
// 		h3lis331dl_accel_convert(val, data->acc[0], sensitivity);
// 		break;
// 	case SENSOR_CHAN_ACCEL_Y:
// 		h3lis331dl_accel_convert(val, data->acc[1], sensitivity);
// 		break;
// 	case SENSOR_CHAN_ACCEL_Z:
// 		h3lis331dl_accel_convert(val, data->acc[2], sensitivity);
// 		break;
// 	case SENSOR_CHAN_ACCEL_XYZ:
// 		for (i = 0; i < 3; i++) {
// 			h3lis331dl_accel_convert(val++, data->acc[i], sensitivity);
// 		}
// 		break;
// 	default:
// 		return -ENOTSUP;
// 	}

// 	return 0;
// }

// static int h3lis331dl_accel_channel_get(enum sensor_channel chan,
// 				     struct sensor_value *val,
// 				     struct h3lis331dl_data *data)
// {
// 	return h3lis331dl_accel_get_channel(chan, val, data, data->acc_gain);
// }

// static inline void h3lis331dl_gyro_convert(struct sensor_value *val, int raw_val,
// 					uint32_t sensitivity)
// {
// 	int64_t dval;

// 	/* Sensitivity is exposed in udps/LSB */
// 	/* So, calculate value in 10 udps unit and then to rad/s */
// 	dval = (int64_t)(raw_val) * sensitivity / 10;
// 	sensor_10udegrees_to_rad(dval, val);
// }

static int h3lis331dl_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
// 	struct h3lis331dl_data *data = dev->data;

// 	switch (chan) {
// 	case SENSOR_CHAN_ACCEL_X:
// 	case SENSOR_CHAN_ACCEL_Y:
// 	case SENSOR_CHAN_ACCEL_Z:
// 	case SENSOR_CHAN_ACCEL_XYZ:
// 		h3lis331dl_accel_channel_get(chan, val, data);
// 		break;
// 	case SENSOR_CHAN_GYRO_X:
// 	case SENSOR_CHAN_GYRO_Y:
// 	case SENSOR_CHAN_GYRO_Z:
// 	case SENSOR_CHAN_GYRO_XYZ:
// 		h3lis331dl_gyro_channel_get(chan, val, data);
// 		break;
// #if defined(CONFIG_H3LIS331DL_ENABLE_TEMP)
// 	case SENSOR_CHAN_DIE_TEMP:
// 		h3lis331dl_gyro_channel_get_temp(val, data);
// 		break;
// #endif
// #if defined(CONFIG_H3LIS331DL_SENSORHUB)
// 	case SENSOR_CHAN_MAGN_X:
// 	case SENSOR_CHAN_MAGN_Y:
// 	case SENSOR_CHAN_MAGN_Z:
// 	case SENSOR_CHAN_MAGN_XYZ:
// 		if (!data->shub_inited) {
// 			LOG_ERR("attr_set() shub not inited.");
// 			return -ENOTSUP;
// 		}

// 		h3lis331dl_magn_get_channel(chan, val, data);
// 		break;

// 	case SENSOR_CHAN_HUMIDITY:
// 		if (!data->shub_inited) {
// 			LOG_ERR("attr_set() shub not inited.");
// 			return -ENOTSUP;
// 		}

// 		h3lis331dl_hum_convert(val, data);
// 		break;

// 	case SENSOR_CHAN_PRESS:
// 		if (!data->shub_inited) {
// 			LOG_ERR("attr_set() shub not inited.");
// 			return -ENOTSUP;
// 		}

// 		h3lis331dl_press_convert(val, data);
// 		break;

// 	case SENSOR_CHAN_AMBIENT_TEMP:
// 		if (!data->shub_inited) {
// 			LOG_ERR("attr_set() shub not inited.");
// 			return -ENOTSUP;
// 		}

// 		h3lis331dl_temp_convert(val, data);
// 		break;
// #endif
// 	default:
// 		return -ENOTSUP;
// 	}

	return 0;
}

static DEVICE_API(sensor, h3lis331dl_driver_api) = {
	.attr_set = h3lis331dl_attr_set,
#if CONFIG_H3LIS331DL_TRIGGER
	.trigger_set = h3lis331dl_trigger_set,
#endif
	.sample_fetch = h3lis331dl_sample_fetch,
	.channel_get = h3lis331dl_channel_get,
};

static int h3lis331dl_init_chip(const struct device *dev)
{
	const struct h3lis331dl_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct h3lis331dl_data *h3lis331dl = dev->data;
	uint8_t chip_id;

	if (h3lis331dl_boot_set(ctx, 1) < 0) {
		LOG_DBG("failed to reboot device");
		return -EIO;
	}

	/* Boot time is min. ODR + 1 ms. Min ODR is 50 Hz --> 51 ms. Give some reserve... */
	k_sleep(K_MSEC(100));

	if (h3lis331dl_device_id_get(ctx, &chip_id) < 0) {
		LOG_DBG("Failed reading chip id");
		return -EIO;
	}

	LOG_INF("chip id 0x%x", chip_id);

	if (chip_id != H3LIS331DL_ID) {
		LOG_DBG("Invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	h3lis331dl_ctrl_reg1_t ctrl_reg1;
	if (h3lis331dl_read_reg(ctx, H3LIS331DL_CTRL_REG1,
				(uint8_t *)&ctrl_reg1,
				sizeof(ctrl_reg1)) < 0) {
		LOG_DBG("Failed to read ctrl_reg1");
		return -EIO;
	}

	/* Set normal power mode */
	ctrl_reg1.pm = 0x01;  // TODO: Add define!
	if (h3lis331dl_write_reg(ctx, H3LIS331DL_CTRL_REG1,
				 (uint8_t *)&ctrl_reg1,
				 sizeof(ctrl_reg1)) < 0) {
		LOG_DBG("Failed to write ctrl_reg1");
		return -EIO;
	}

	if (h3lis331dl_axis_x_data_set(ctx, 1) < 0) {  // TODO: Add define!
		LOG_DBG("Failed to enable x axis");
		return -EIO;
	}

	if (h3lis331dl_axis_y_data_set(ctx, 1) < 0) {  // TODO: Add define!
		LOG_DBG("Failed to enable y axis");
		return -EIO;
	}

	if (h3lis331dl_axis_z_data_set(ctx, 1) < 0) {  // TODO: Add define!
		LOG_DBG("Failed to enable z axis");
		return -EIO;
	}


















	// /* set accel power mode */
	// LOG_DBG("accel pm is %d", cfg->accel_pm);
	// switch (cfg->accel_pm) {
	// default:
	// case 0:
	// 	h3lis331dl_xl_power_mode_set(ctx, H3LIS331DL_HIGH_PERFORMANCE_MD);
	// 	break;
	// case 1:
	// 	h3lis331dl_xl_power_mode_set(ctx, H3LIS331DL_LOW_NORMAL_POWER_MD);
	// 	break;
	// case 2:
	// 	h3lis331dl_xl_power_mode_set(ctx, H3LIS331DL_ULTRA_LOW_POWER_MD);
	// 	break;
	// }

	// fs = cfg->accel_range & ACCEL_RANGE_MASK;
	// LOG_DBG("accel range is %d", fs);
	// if (h3lis331dl_accel_set_fs_raw(dev, fs) < 0) {
	// 	LOG_ERR("failed to set accelerometer range %d", fs);
	// 	return -EIO;
	// }
	// h3lis331dl->acc_gain = h3lis331dl_accel_fs_val_to_gain(fs, cfg->accel_range & ACCEL_RANGE_DOUBLE);

	// odr = cfg->accel_odr;
	// LOG_DBG("accel odr is %d", odr);
	// h3lis331dl->accel_freq = h3lis331dl_odr_to_freq_val(odr);
	// if (h3lis331dl_accel_set_odr_raw(dev, odr) < 0) {
	// 	LOG_ERR("failed to set accelerometer odr %d", odr);
	// 	return -EIO;
	// }

	// /* Set FIFO bypass mode */
	// if (h3lis331dl_fifo_mode_set(ctx, H3LIS331DL_BYPASS_MODE) < 0) {
	// 	LOG_DBG("failed to set FIFO mode");
	// 	return -EIO;
	// }

	// if (h3lis331dl_block_data_update_set(ctx, 1) < 0) {
	// 	LOG_DBG("failed to set BDU mode");
	// 	return -EIO;
	// }

	return 0;
}

static int h3lis331dl_init(const struct device *dev)
{
// #ifdef CONFIG_H3LIS331DL_TRIGGER
// 	const struct h3lis331dl_config *cfg = dev->config;
// #endif
	struct h3lis331dl_data *data = dev->data;

	LOG_INF("Initialize device %s", dev->name);
	data->dev = dev;

	if (h3lis331dl_init_chip(dev) < 0) {
		LOG_DBG("failed to initialize chip");
		return -EIO;
	}

// #ifdef CONFIG_H3LIS331DL_TRIGGER
// 	if (cfg->trig_enabled) {
// 		if (h3lis331dl_init_interrupt(dev) < 0) {
// 			LOG_ERR("Failed to initialize interrupt.");
// 			return -EIO;
// 		}
// 	}
// #endif

	return 0;
}

/*
 * Device creation macro, shared by H3LIS331DL_DEFINE_SPI() and
 * H3LIS331DL_DEFINE_I2C().
 */

#define H3LIS331DL_DEVICE_INIT(inst, model)				\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			    h3lis331dl_init,				\
			    NULL,					\
			    &model##_data_##inst,			\
			    &model##_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &h3lis331dl_driver_api);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#ifdef CONFIG_H3LIS331DL_TRIGGER
#define H3LIS331DL_CFG_IRQ(inst)						\
	.trig_enabled = true,						\
	.gpio_drdy = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),		\
	.int_pin = DT_INST_PROP(inst, int_pin)
#else
#define H3LIS331DL_CFG_IRQ(inst)
#endif /* CONFIG_H3LIS331DL_TRIGGER */

#define H3LIS331DL_SPI_OP  (SPI_WORD_SET(8) |				\
			 SPI_OP_MODE_MASTER |				\
			 SPI_MODE_CPOL |				\
			 SPI_MODE_CPHA)					\

#define H3LIS331DL_CONFIG_COMMON(inst)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),		\
		(H3LIS331DL_CFG_IRQ(inst)), ())

#define H3LIS331DL_CONFIG_SPI(inst, model)					\
	{								\
		STMEMSC_CTX_SPI(&model##_config_##inst.stmemsc_cfg),	\
		.stmemsc_cfg = {					\
			.spi = SPI_DT_SPEC_INST_GET(inst,		\
					   H3LIS331DL_SPI_OP),		\
		},							\
		H3LIS331DL_CONFIG_COMMON(inst)				\
	}

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define H3LIS331DL_CONFIG_I2C(inst, model)					\
	{								\
		STMEMSC_CTX_I2C(&model##_config_##inst.stmemsc_cfg),	\
		.stmemsc_cfg = {					\
			.i2c = I2C_DT_SPEC_INST_GET(inst),		\
		},							\
		iwH3LIS331DH3LIS331DL_CONFIG_COMMON(inst)				\
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define H3LIS331DL_DEFINE(inst, model)					\
	static struct h3lis331dl_data model##_data_##inst;			\
	static const struct h3lis331dl_config model##_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			(H3LIS331DL_CONFIG_SPI(inst, model)),		\
			(H3LIS331DL_CONFIG_I2C(inst, model)));		\
	H3LIS331DL_DEVICE_INIT(inst, model)

#define DT_DRV_COMPAT st_h3lis331dl
DT_INST_FOREACH_STATUS_OKAY_VARGS(H3LIS331DL_DEFINE, h3lis331dl)
#undef DT_DRV_COMPAT
