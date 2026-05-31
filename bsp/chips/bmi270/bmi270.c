/**
\brief bmi270 driver.

\author OpenWSN contributors, May 2026.
*/

#include "i2c.h"
#include "bmi270.h"

//=========================== define ==========================================

typedef struct{

    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;

}bmi270_data_t;

typedef struct {

    bmi270_data_t bmi270_data;

}bmi270_var_t;

//=========================== variables =======================================

bmi270_var_t bmi270_var;

//=========================== prototypes ======================================

//=========================== public ==========================================

// admin
uint8_t bmi270_who_am_i(void) {

    uint8_t chipid;
    chipid = 0;
    i2c_read_bytes(BMI270_REG_ADDR_CHIPID, &chipid, 1);
    return chipid;
}

uint8_t bmi270_get_status(void) {

    uint8_t status;
    status = 0;
    i2c_read_bytes(BMI270_REG_ADDR_STATUS, &status, 1);
    return status;
}

uint8_t bmi270_get_errorreg(void) {

    uint8_t error_reg;
    error_reg = 0;
    i2c_read_bytes(BMI270_REG_ADDR_ERR_REG, &error_reg, 1);
    return error_reg;
}

uint8_t bmi270_get_internal_status(void) {

    uint8_t status;
    status = 0;
    i2c_read_bytes(BMI270_REG_ADDR_INTERNAL_STATUS, &status, 1);
    return status;
}

void bmi270_set_cmd(uint8_t cmd) {

    i2c_write_bytes(BMI270_REG_ADDR_CMD, &cmd, 1);
}

void bmi270_soft_reset(void) {

    bmi270_set_cmd(BMI270_CMD_SOFT_RESET);
}

void bmi270_power_on(void) {

    uint8_t power_conf;
    uint8_t power_ctrl;

    power_conf = BMI270_PWR_CONF_PERF_MODE;
    power_ctrl = BMI270_PWR_CTRL_ACC_GYR_EN | BMI270_PWR_CTRL_TEMP_EN;

    i2c_write_bytes(BMI270_REG_ADDR_PWR_CONF, &power_conf, 1);
    i2c_write_bytes(BMI270_REG_ADDR_PWR_CTRL, &power_ctrl, 1);
}

void bmi270_power_down(void) {

    uint8_t power_ctrl;

    power_ctrl = 0x00;
    i2c_write_bytes(BMI270_REG_ADDR_PWR_CTRL, &power_ctrl, 1);
}

// configuration

void bmi270_acc_config(uint8_t config) {

    i2c_write_bytes(BMI270_REG_ADDR_ACC_CONF, &config, 1);
}

void bmi270_gyr_config(uint8_t config) {

    i2c_write_bytes(BMI270_REG_ADDR_GYR_CONF, &config, 1);
}

void bmi270_acc_range(uint8_t range) {

    i2c_write_bytes(BMI270_REG_ADDR_ACC_RANGE, &range, 1);
}

void bmi270_gyr_range(uint8_t range) {

    i2c_write_bytes(BMI270_REG_ADDR_GYR_RANGE, &range, 1);
}

void bmi270_default_config(void) {

    bmi270_acc_config(BMI270_ACC_CONF_DEFAULT);
    bmi270_gyr_config(BMI270_GYR_CONF_DEFAULT);
    bmi270_acc_range(BMI270_ACC_RANGE_8G);
    bmi270_gyr_range(BMI270_GYR_RANGE_1000DPS);
    bmi270_power_on();
}

// read
void bmi270_read_6dof_data(void) {

    i2c_read_bytes(
        BMI270_REG_ADDR_DATA,
        (uint8_t*)(&bmi270_var.bmi270_data),
        sizeof(bmi270_data_t)
    );
}

int16_t bmi270_read_acc_x(void) {

    return bmi270_var.bmi270_data.acc_x;
}

int16_t bmi270_read_acc_y(void) {

    return bmi270_var.bmi270_data.acc_y;
}

int16_t bmi270_read_acc_z(void) {

    return bmi270_var.bmi270_data.acc_z;
}

int16_t bmi270_read_gyr_x(void) {

    return bmi270_var.bmi270_data.gyr_x;
}

int16_t bmi270_read_gyr_y(void) {

    return bmi270_var.bmi270_data.gyr_y;
}

int16_t bmi270_read_gyr_z(void) {

    return bmi270_var.bmi270_data.gyr_z;
}

int16_t bmi270_read_temperature(void) {

    uint8_t temp_l;
    uint8_t temp_h;

    i2c_read_bytes(BMI270_REG_ADDR_TEMPERATURE_0, &temp_l, 1);
    i2c_read_bytes(BMI270_REG_ADDR_TEMPERATURE_1, &temp_h, 1);

    return (int16_t)(((uint16_t)temp_h << 8) | temp_l);
}

//=========================== private =========================================
