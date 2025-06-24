#include <nrfx_twim.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// I2C Device Addresses
#define BM1422AGMV_ADDR0						(0x0e)
#define BM1422AGMV_ADDR1						(0x0f)

// Register List for BM1422AGMV
#define BM1422AGMV_WIA_VAL						(0x41)

#define BM1422AGMV_WIA							(0x0f)
#define BM1422AGMV_DATAX						(0x10)
#define BM1422AGMV_STA1							(0x18)
#define BM1422AGMV_CNTL1						(0x1b)
#define BM1422AGMV_CNTL2						(0x1c)
#define BM1422AGMV_CNTL3						(0x1d)
#define BM1422AGMV_AVE_A						(0x40)
#define BM1422AGMV_CNTL4						(0x5c)

#define BM1422AGMV_STA1_RD_DRDY					(1 << 6)

#define BM1422AGMV_CNTL1_FS1					(1 << 1)
#define BM1422AGMV_CNTL1_ODR_10Hz				(0 << 3)
#define BM1422AGMV_CNTL1_ODR_20Hz				(2 << 3)
#define BM1422AGMV_CNTL1_ODR_100Hz				(1 << 3)
#define BM1422AGMV_CNTL1_ODR_1000Hz				(3 << 3)
#define BM1422AGMV_CNTL1_RST_LV					(1 << 5)
#define BM1422AGMV_CNTL1_OUT_BIT				(1 << 6)
#define BM1422AGMV_CNTL1_PC1					(1 << 7)

#define BM1422AGMV_CNTL2_DRP					(1 << 2)
#define BM1422AGMV_CNTL2_DREN					(1 << 3)

#define BM1422AGMV_CNTL3_FORCE					(1 << 6)

#define BM1422AGMV_AVE_A_AVE4					(0 << 2)

#define BM1422AGMV_14BIT_SENS					(24)
#define BM1422AGMV_12BIT_SENS					(6)
