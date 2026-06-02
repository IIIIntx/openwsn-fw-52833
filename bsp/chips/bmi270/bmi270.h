/**
\brief registers address mapping of bmi270 sensor.

\author OpenWSN contributors, May 2026.
*/

#ifndef __BMI270_H
#define __BMI270_H

#include "stdint.h"

//=========================== define ==========================================

#define BMI270_ADDR                         0x68
#define BMI270_ADDR_ALT                     0x69

//---- register addresses

#define BMI270_REG_ADDR_CHIPID              0x00
#define BMI270_REG_ADDR_ERR_REG             0x02
#define BMI270_REG_ADDR_STATUS              0x03

// auxiliary sensor data

#define BMI270_REG_ADDR_AUX_X_L             0x04
#define BMI270_REG_ADDR_AUX_X_H             0x05
#define BMI270_REG_ADDR_AUX_Y_L             0x06
#define BMI270_REG_ADDR_AUX_Y_H             0x07
#define BMI270_REG_ADDR_AUX_Z_L             0x08
#define BMI270_REG_ADDR_AUX_Z_H             0x09
#define BMI270_REG_ADDR_AUX_R_L             0x0a
#define BMI270_REG_ADDR_AUX_R_H             0x0b

// accelerometer data

#define BMI270_REG_ADDR_ACC_X_L             0x0c
#define BMI270_REG_ADDR_ACC_X_H             0x0d
#define BMI270_REG_ADDR_ACC_Y_L             0x0e
#define BMI270_REG_ADDR_ACC_Y_H             0x0f
#define BMI270_REG_ADDR_ACC_Z_L             0x10
#define BMI270_REG_ADDR_ACC_Z_H             0x11

// gyroscope data

#define BMI270_REG_ADDR_GYR_X_L             0x12
#define BMI270_REG_ADDR_GYR_X_H             0x13
#define BMI270_REG_ADDR_GYR_Y_L             0x14
#define BMI270_REG_ADDR_GYR_Y_H             0x15
#define BMI270_REG_ADDR_GYR_Z_L             0x16
#define BMI270_REG_ADDR_GYR_Z_H             0x17

#define BMI270_REG_ADDR_DATA                BMI270_REG_ADDR_ACC_X_L
#define BMI270_REG_ADDR_SENSORTIME_0        0x18
#define BMI270_REG_ADDR_SENSORTIME_1        0x19
#define BMI270_REG_ADDR_SENSORTIME_2        0x1a
#define BMI270_REG_ADDR_EVENT               0x1b
#define BMI270_REG_ADDR_INT_STATUS_0        0x1c
#define BMI270_REG_ADDR_INT_STATUS_1        0x1d
#define BMI270_REG_ADDR_INTERNAL_STATUS     0x21
#define BMI270_REG_ADDR_TEMPERATURE_0       0x22
#define BMI270_REG_ADDR_TEMPERATURE_1       0x23
#define BMI270_REG_ADDR_FIFO_LENGTH_0       0x24
#define BMI270_REG_ADDR_FIFO_LENGTH_1       0x25
#define BMI270_REG_ADDR_FIFO_DATA           0x26
#define BMI270_REG_ADDR_FEAT_PAGE           0x2f
#define BMI270_REG_ADDR_FEATURES_0          0x30
#define BMI270_REG_ADDR_ACC_CONF            0x40
#define BMI270_REG_ADDR_ACC_RANGE           0x41
#define BMI270_REG_ADDR_GYR_CONF            0x42
#define BMI270_REG_ADDR_GYR_RANGE           0x43
#define BMI270_REG_ADDR_AUX_CONF            0x44
#define BMI270_REG_ADDR_FIFO_DOWNS          0x45
#define BMI270_REG_ADDR_FIFO_WTM_0          0x46
#define BMI270_REG_ADDR_FIFO_WTM_1          0x47
#define BMI270_REG_ADDR_FIFO_CONFIG_0       0x48
#define BMI270_REG_ADDR_FIFO_CONFIG_1       0x49
#define BMI270_REG_ADDR_AUX_DEV_ID          0x4b
#define BMI270_REG_ADDR_AUX_IF_CONF         0x4c
#define BMI270_REG_ADDR_AUX_RD_ADDR         0x4d
#define BMI270_REG_ADDR_AUX_WR_ADDR         0x4e
#define BMI270_REG_ADDR_AUX_WR_DATA         0x4f
#define BMI270_REG_ADDR_INT1_IO_CTRL        0x53
#define BMI270_REG_ADDR_INT2_IO_CTRL        0x54
#define BMI270_REG_ADDR_INT_LATCH           0x55
#define BMI270_REG_ADDR_INT1_MAP_FEAT       0x56
#define BMI270_REG_ADDR_INT2_MAP_FEAT       0x57
#define BMI270_REG_ADDR_INT_MAP_DATA        0x58
#define BMI270_REG_ADDR_INIT_CTRL           0x59
#define BMI270_REG_ADDR_INIT_ADDR_0         0x5b
#define BMI270_REG_ADDR_INIT_ADDR_1         0x5c
#define BMI270_REG_ADDR_INIT_DATA           0x5e
#define BMI270_REG_ADDR_INTERNAL_ERROR      0x5f
#define BMI270_REG_ADDR_GYR_CRT_CONF        0x69
#define BMI270_REG_ADDR_NVM_CONF            0x6a
#define BMI270_REG_ADDR_IF_CONF             0x6b
#define BMI270_REG_ADDR_DRV                 0x6c
#define BMI270_REG_ADDR_ACC_SELF_TEST       0x6d
#define BMI270_REG_ADDR_GYR_SELF_TEST_AXES  0x6e
#define BMI270_REG_ADDR_NV_CONF             0x70
#define BMI270_REG_ADDR_OFFSET_0            0x71
#define BMI270_REG_ADDR_OFFSET_1            0x72
#define BMI270_REG_ADDR_OFFSET_2            0x73
#define BMI270_REG_ADDR_OFFSET_3            0x74
#define BMI270_REG_ADDR_OFFSET_4            0x75
#define BMI270_REG_ADDR_OFFSET_5            0x76
#define BMI270_REG_ADDR_OFFSET_6            0x77
#define BMI270_REG_ADDR_PWR_CONF            0x7c
#define BMI270_REG_ADDR_PWR_CTRL            0x7d
#define BMI270_REG_ADDR_CMD                 0x7e

//---- register values

#define BMI270_CHIPID                       0x24

#define BMI270_CMD_SOFT_RESET               0xb6
#define BMI270_CMD_FIFO_FLUSH               0xb0
#define BMI270_CMD_GYR_SELF_TEST            0x03

#define BMI270_INIT_CTRL_LOAD_DONE          0x01
#define BMI270_INTERNAL_STATUS_INIT_OK      0x01

#define BMI270_PWR_CONF_ADV_POWER_SAVE      0x01
#define BMI270_PWR_CONF_PERF_MODE           0x00

#define BMI270_PWR_CTRL_AUX_EN              0x01
#define BMI270_PWR_CTRL_GYR_EN              0x02
#define BMI270_PWR_CTRL_ACC_EN              0x04
#define BMI270_PWR_CTRL_TEMP_EN             0x08
#define BMI270_PWR_CTRL_ACC_GYR_EN          (BMI270_PWR_CTRL_ACC_EN | BMI270_PWR_CTRL_GYR_EN)
#define BMI270_PWR_CTRL_ALL_EN              (BMI270_PWR_CTRL_AUX_EN | BMI270_PWR_CTRL_GYR_EN | BMI270_PWR_CTRL_ACC_EN | BMI270_PWR_CTRL_TEMP_EN)

// ACC_CONF/GYR_CONF ODR field values

#define BMI270_ODR_12P5HZ                   0x05
#define BMI270_ODR_25HZ                     0x06
#define BMI270_ODR_50HZ                     0x07
#define BMI270_ODR_100HZ                    0x08
#define BMI270_ODR_200HZ                    0x09
#define BMI270_ODR_400HZ                    0x0a
#define BMI270_ODR_800HZ                    0x0b
#define BMI270_ODR_1600HZ                   0x0c

#define BMI270_ACC_BWP_OSR4                 0x00
#define BMI270_ACC_BWP_OSR2                 0x01
#define BMI270_ACC_BWP_NORMAL_AVG4          0x02
#define BMI270_ACC_BWP_CIC_AVG8             0x03
#define BMI270_ACC_BWP_RES_AVG16            0x04
#define BMI270_ACC_BWP_RES_AVG32            0x05
#define BMI270_ACC_BWP_RES_AVG64            0x06
#define BMI270_ACC_BWP_RES_AVG128           0x07
#define BMI270_ACC_FILTER_PERF_POWER        0x00
#define BMI270_ACC_FILTER_PERF_HIGH         0x01

#define BMI270_GYR_BWP_OSR4                 0x00
#define BMI270_GYR_BWP_OSR2                 0x01
#define BMI270_GYR_BWP_NORMAL               0x02
#define BMI270_GYR_NOISE_PERF_POWER         0x00
#define BMI270_GYR_NOISE_PERF_HIGH          0x01
#define BMI270_GYR_FILTER_PERF_POWER        0x00
#define BMI270_GYR_FILTER_PERF_HIGH         0x01

// range values

#define BMI270_ACC_RANGE_2G                 0x00
#define BMI270_ACC_RANGE_4G                 0x01
#define BMI270_ACC_RANGE_8G                 0x02
#define BMI270_ACC_RANGE_16G                0x03

#define BMI270_GYR_RANGE_2000DPS            0x00
#define BMI270_GYR_RANGE_1000DPS            0x01
#define BMI270_GYR_RANGE_500DPS             0x02
#define BMI270_GYR_RANGE_250DPS             0x03
#define BMI270_GYR_RANGE_125DPS             0x04

#define BMI270_ACC_CONF(odr, bwp, perf)     (((perf) << 7) | ((bwp) << 4) | (odr))
#define BMI270_GYR_CONF(odr, bwp, noise, filter) \
                                            (((filter) << 7) | ((noise) << 6) | ((bwp) << 4) | (odr))

#define BMI270_ACC_CONF_DEFAULT             BMI270_ACC_CONF(BMI270_ODR_100HZ, BMI270_ACC_BWP_NORMAL_AVG4, BMI270_ACC_FILTER_PERF_HIGH)
#define BMI270_GYR_CONF_DEFAULT             BMI270_GYR_CONF(BMI270_ODR_100HZ, BMI270_GYR_BWP_NORMAL, BMI270_GYR_NOISE_PERF_HIGH, BMI270_GYR_FILTER_PERF_HIGH)

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

// admin
uint8_t bmi270_who_am_i(void);
uint8_t bmi270_get_status(void);
uint8_t bmi270_get_errorreg(void);
uint8_t bmi270_get_internal_status(void);

void    bmi270_set_cmd(uint8_t cmd);
void    bmi270_soft_reset(void);
void    bmi270_power_on(void);
void    bmi270_power_down(void);

void    bmi270_acc_config(uint8_t config);
void    bmi270_gyr_config(uint8_t config);
void    bmi270_acc_range(uint8_t range);
void    bmi270_gyr_range(uint8_t range);
uint8_t bmi270_default_config(void);

// read
uint32_t bmi270_last_i2c_result(void);
uint32_t bmi270_read_6dof_data(void);

int16_t bmi270_read_temperature(void);

int16_t bmi270_read_acc_x(void);
int16_t bmi270_read_acc_y(void);
int16_t bmi270_read_acc_z(void);

int16_t bmi270_read_gyr_x(void);
int16_t bmi270_read_gyr_y(void);
int16_t bmi270_read_gyr_z(void);

//=========================== private =========================================

#endif
