/*
 * Copyright (c) 2024 Xenoma Inc.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ICM-45686.h"
#include "bm1422a.h"
#include "tca9548a.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// for Semaphore
K_SEM_DEFINE(sem_sensor, 0, 1);

/* ----- TWIM Function Start ----- */
static const struct device *dev_twim0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static int twim_write(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t num_bytes)
{
	int err;
	struct i2c_msg msgs[2];

	/* Send the address to write to */
	msgs[0].buf = &reg;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err = i2c_transfer(dev, &msgs[0], 2, addr);
	return err;
}

static int twim_write_noreg(const struct device *dev, uint8_t addr, uint8_t *data, uint32_t num_bytes)
{
	int err;
	struct i2c_msg msgs;

	/* Data to be written, and STOP after this. */
	msgs.buf = data;
	msgs.len = num_bytes;
	msgs.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err = i2c_transfer(dev, &msgs, 1, addr);
	k_sleep(K_USEC(10));
	return err;
}

static int twim_read(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t num_bytes)
{
	int err;
	struct i2c_msg msgs[2] = {0};

	/* Send the address to read from */
	msgs[0].buf = &reg;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	err = i2c_transfer(dev, &msgs[0], 2, addr);
	return err;
}
/* ----- TWIM Function End ----- */

/* ----- Device Function Start ----- */
// Initialize BM1422AGMV TWIM
bool BM1422A_init(const struct device *dev, uint8_t device_addr)
{
	const uint8_t INIT_CMD[] = {
		BM1422AGMV_CNTL1,
		BM1422AGMV_CNTL4,
		BM1422AGMV_CNTL4 + 1,
		BM1422AGMV_CNTL2,
		BM1422AGMV_AVE_A,
		BM1422AGMV_CNTL3,
	};

	const uint8_t INIT_PARAM[] = {
		(BM1422AGMV_CNTL1_ODR_100Hz | BM1422AGMV_CNTL1_OUT_BIT | BM1422AGMV_CNTL1_PC1),
		0x00,
		0x00,
		0x00,
		BM1422AGMV_AVE_A_AVE4,
		BM1422AGMV_CNTL3_FORCE,
	};
	int err;
    uint8_t twim_buffer[16] = {0};

    // Send parameter
    for (uint8_t i = 0; i < sizeof(INIT_CMD); i++) {
		twim_buffer[0] = INIT_PARAM[i];
		err = twim_write(dev, device_addr, INIT_CMD[i], twim_buffer, 1);
		if (err < 0) {
			LOG_ERR("TWIM Error!(%d)", err);
			return false;
		}
    }

    return true;
}

static bool ICM45686_init(const struct device *dev, uint8_t device_addr)
{
	const uint8_t INIT_CMD[] = {
		REG_MISC2,
		WAIT_MSEC,
		ACCEL_CONFIG0,
		GYRO_CONFIG0,
		IOC_PAD_SCENARIO_AUX_OVRD,
		PWR_MGMT0,
	};
	const uint8_t INIT_PARAM[] = {
		0x02,
		0x0A,
		0x19,
		0x19,
		0x02,
		0x0F,
	};
	int err;
	uint8_t twim_buffer[8] = {0};

	// Check IMU sensor
	err = twim_read(dev, device_addr, WHO_AM_I, twim_buffer, 1);
	if (err < 0) {
		LOG_ERR("Device cannot be found.(%d)", err);
		return false;
	} else if (twim_buffer[0] != I_AM_ICM45686) {
		LOG_ERR("This device is not ICM-45686.");
		return false;
	}

	// Send
	for (uint8_t i = 0; i < sizeof(INIT_CMD); i++) {
		if (INIT_CMD[i] == WAIT_MSEC) {
			k_sleep(K_MSEC(INIT_PARAM[i]));
		} else {
			twim_buffer[0] = INIT_PARAM[i];
			err = twim_write(dev, device_addr, INIT_CMD[i], twim_buffer, 1);
			if (err < 0) {
				LOG_ERR("TWIM Error!(%d)", err);
				return false;
			}
		}
	}
	return true;
}
/* ----- Device Function End ----- */

/* ----- Timer Function Start ----- */
static void read_sensor_work_thread(void)
{
	int err;
	uint8_t twim_buffer[16];
	int16_t accel[3], gyro[3], mag[3];

	while (true) {
		k_sem_take(&sem_sensor, K_FOREVER);

		// ID0
		err = twim_read(dev_twim0, ICM45686_ADDR0, ACCEL_DATA_X1_UI, twim_buffer, 12);
		if (err < 0) {
			LOG_WRN("Failed to read sensor ID0.");
		} else {
			accel[0] = (int16_t)(twim_buffer[1] << 8) | twim_buffer[0];
			accel[1] = (int16_t)(twim_buffer[3] << 8) | twim_buffer[2];
			accel[2] = (int16_t)(twim_buffer[5] << 8) | twim_buffer[4];
			gyro[0] = (int16_t)(twim_buffer[7] << 8) | twim_buffer[6];
			gyro[1] = (int16_t)(twim_buffer[9] << 8) | twim_buffer[8];
			gyro[2] = (int16_t)(twim_buffer[11] << 8) | twim_buffer[10];
			// Display value(s)
			LOG_INF("ACC XYZ ID0 = 0x%04x, 0x%04x, 0x%04x", accel[0], accel[1], accel[2]);
			LOG_INF("GYR XYZ ID0 = 0x%04x, 0x%04x, 0x%04x", gyro[0], gyro[1], gyro[2]);
		}

		err = twim_read(dev_twim0, BM1422AGMV_ADDR0, BM1422AGMV_DATAX, twim_buffer, 6);
		if (err < 0) {
			LOG_WRN("Failed to read sensor ID0.");
		} else {
			mag[0] = (int16_t)(twim_buffer[1] << 8) | twim_buffer[0];
			mag[1] = (int16_t)(twim_buffer[3] << 8) | twim_buffer[2];
			mag[2] = (int16_t)(twim_buffer[5] << 8) | twim_buffer[4];
			// Display value(s)
			LOG_INF("MAG XYZ ID0 = 0x%04x, 0x%04x, 0x%04x", mag[0], mag[1], mag[2]);
		}
		LOG_INF("-----");

		err = twim_read(dev_twim0, ICM45686_ADDR1, ACCEL_DATA_X1_UI, twim_buffer, 12);
		if (err < 0) {
			LOG_WRN("Failed to read sensor ID1.");
		} else {
			accel[0] = (int16_t)(twim_buffer[1] << 8) | twim_buffer[0];
			accel[1] = (int16_t)(twim_buffer[3] << 8) | twim_buffer[2];
			accel[2] = (int16_t)(twim_buffer[5] << 8) | twim_buffer[4];
			gyro[0] = (int16_t)(twim_buffer[7] << 8) | twim_buffer[6];
			gyro[1] = (int16_t)(twim_buffer[9] << 8) | twim_buffer[8];
			gyro[2] = (int16_t)(twim_buffer[11] << 8) | twim_buffer[10];
			// Display value(s)
			LOG_INF("ACC XYZ ID1 = 0x%04x, 0x%04x, 0x%04x", accel[0], accel[1], accel[2]);
			LOG_INF("GYR XYZ ID1 = 0x%04x, 0x%04x, 0x%04x", gyro[0], gyro[1], gyro[2]);
		}

		err = twim_read(dev_twim0, BM1422AGMV_ADDR1, BM1422AGMV_DATAX, twim_buffer, 6);
		if (err < 0) {
			LOG_WRN("Failed to read sensor ID1.");
		} else {
			mag[0] = (int16_t)(twim_buffer[1] << 8) | twim_buffer[0];
			mag[1] = (int16_t)(twim_buffer[3] << 8) | twim_buffer[2];
			mag[2] = (int16_t)(twim_buffer[5] << 8) | twim_buffer[4];
			// Display value(s)
			LOG_INF("MAG XYZ ID1 = 0x%04x, 0x%04x, 0x%04x", mag[0], mag[1], mag[2]);
		}
		LOG_INF("-----");
	}
}
K_THREAD_DEFINE(read_sensor, 1024, read_sensor_work_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(15), 0, 0);

static void timer_stop_handler(struct k_timer *timer)
{
	LOG_INF("Timer stopped.");
}

static void timer_handler(struct k_timer *timer)
{
	// Resume thread
	LOG_DBG("Read sensor(s).");
	k_sem_give(&sem_sensor);
}
K_TIMER_DEFINE(timer, timer_handler, timer_stop_handler);
/* ----- Timer Function End ----- */

int main(void)
{
	int err;
	uint8_t twim_buffer[16] = {0};

	twim_buffer[0] = CHANNEL0_ENABLED;
	err = twim_write_noreg(dev_twim0, TCA9548A_ADDR0, twim_buffer, 1);
	if (err) {
		LOG_WRN("Failed to communicate MUX.");
		return -1;
	}

	//
	if (ICM45686_init(dev_twim0, ICM45686_ADDR0) == false) {
		LOG_WRN("Failed to initialize sensor ID0.");
		return -1;
	}
	if (BM1422A_init(dev_twim0, BM1422AGMV_ADDR0)) {
		LOG_INF("[TWIM 0]MAG sensor ID0 has been initialized.");
	} else {
		LOG_ERR("[TWIM 0]Failed to initialize MAG sensor ID0");
		return -1;
	}
	if (ICM45686_init(dev_twim0, ICM45686_ADDR1) == false) {
		LOG_WRN("Failed to initialize sensor ID1.");
		return -1;
	}
	if (BM1422A_init(dev_twim0, BM1422AGMV_ADDR1)) {
		LOG_INF("[TWIM 0]MAG sensor ID1 has been initialized.");
	} else {
		LOG_ERR("[TWIM 0]Failed to initialize MAG sensor ID1");
		return -1;
	}

	//
	k_timer_start(&timer, K_NO_WAIT, K_MSEC(3000));

	// Main loop
	while (true) {
		k_sleep(K_MSEC(500));
	}
}
