/**
\brief bmi270 driver.

\author OpenWSN contributors, May 2026.
*/

#include "i2c.h"
#include "bmi270.h"
#include "bmi270_config.h"

//=========================== define ==========================================

#define BMI270_INIT_CONFIG_CHUNK_LEN        32
#define BMI270_INIT_STATUS_POLL_MAX         20
#define BMI270_DELAY_SOFT_RESET_CYCLES      200000
#define BMI270_DELAY_CONFIG_LOAD_CYCLES     400000
#define BMI270_DELAY_POLL_CYCLES            50000

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
    uint32_t      last_i2c_result;

}bmi270_var_t;

//=========================== variables =======================================

bmi270_var_t bmi270_var;

//=========================== prototypes ======================================

void     bmi270_delay(uint32_t cycles);
uint8_t  bmi270_write_reg(uint8_t reg_addr, uint8_t value);
uint8_t  bmi270_write_init_addr(uint16_t offset);
uint8_t  bmi270_load_config_file(void);

//=========================== public ==========================================

// admin
uint8_t bmi270_who_am_i(void) {

    uint8_t chipid;
    chipid = 0;
    bmi270_var.last_i2c_result = i2c_read_bytes(BMI270_REG_ADDR_CHIPID, &chipid, 1);
    return chipid;
}

uint8_t bmi270_get_status(void) {

    uint8_t status;
    status = 0;
    bmi270_var.last_i2c_result = i2c_read_bytes(BMI270_REG_ADDR_STATUS, &status, 1);
    return status;
}

uint8_t bmi270_get_errorreg(void) {

    uint8_t error_reg;
    error_reg = 0;
    bmi270_var.last_i2c_result = i2c_read_bytes(BMI270_REG_ADDR_ERR_REG, &error_reg, 1);
    return error_reg;
}

uint8_t bmi270_get_internal_status(void) {

    uint8_t status;
    status = 0;
    bmi270_var.last_i2c_result = i2c_read_bytes(BMI270_REG_ADDR_INTERNAL_STATUS, &status, 1);
    return status;
}

void bmi270_set_cmd(uint8_t cmd) {

    bmi270_write_reg(BMI270_REG_ADDR_CMD, cmd);
}

void bmi270_soft_reset(void) {

    bmi270_set_cmd(BMI270_CMD_SOFT_RESET);
}

void bmi270_power_on(void) {

    uint8_t power_conf;
    uint8_t power_ctrl;

    power_conf = BMI270_PWR_CONF_PERF_MODE;
    power_ctrl = BMI270_PWR_CTRL_ACC_GYR_EN | BMI270_PWR_CTRL_TEMP_EN;

    bmi270_write_reg(BMI270_REG_ADDR_PWR_CONF, power_conf);
    bmi270_write_reg(BMI270_REG_ADDR_PWR_CTRL, power_ctrl);
}

void bmi270_power_down(void) {

    uint8_t power_ctrl;

    power_ctrl = 0x00;
    bmi270_write_reg(BMI270_REG_ADDR_PWR_CTRL, power_ctrl);
}

// configuration

void bmi270_acc_config(uint8_t config) {

    bmi270_write_reg(BMI270_REG_ADDR_ACC_CONF, config);
}

void bmi270_gyr_config(uint8_t config) {

    bmi270_write_reg(BMI270_REG_ADDR_GYR_CONF, config);
}

void bmi270_acc_range(uint8_t range) {

    bmi270_write_reg(BMI270_REG_ADDR_ACC_RANGE, range);
}

void bmi270_gyr_range(uint8_t range) {

    bmi270_write_reg(BMI270_REG_ADDR_GYR_RANGE, range);
}

uint8_t bmi270_default_config(void) {

    uint8_t internal_status;

    bmi270_soft_reset();
    bmi270_delay(BMI270_DELAY_SOFT_RESET_CYCLES);

    if (bmi270_who_am_i()!=BMI270_CHIPID) {
        return 0;
    }

    if (bmi270_load_config_file()==0) {
        return 0;
    }

    internal_status = bmi270_get_internal_status();
    if ((internal_status & 0x0f)!=BMI270_INTERNAL_STATUS_INIT_OK) {
        return 0;
    }

    bmi270_acc_config(BMI270_ACC_CONF_DEFAULT);
    bmi270_gyr_config(BMI270_GYR_CONF_DEFAULT);
    bmi270_acc_range(BMI270_ACC_RANGE_8G);
    bmi270_gyr_range(BMI270_GYR_RANGE_1000DPS);
    bmi270_power_on();

    return 1;
}

// read
uint32_t bmi270_last_i2c_result(void) {

    return bmi270_var.last_i2c_result;
}

uint32_t bmi270_read_6dof_data(void) {

    bmi270_var.last_i2c_result = i2c_read_bytes(
        BMI270_REG_ADDR_DATA,
        (uint8_t*)(&bmi270_var.bmi270_data),
        sizeof(bmi270_data_t)
    );

    return bmi270_var.last_i2c_result;
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

    bmi270_var.last_i2c_result = i2c_read_bytes(BMI270_REG_ADDR_TEMPERATURE_0, &temp_l, 1);
    if (bmi270_var.last_i2c_result==0) {
        return 0;
    }
    bmi270_var.last_i2c_result = i2c_read_bytes(BMI270_REG_ADDR_TEMPERATURE_1, &temp_h, 1);
    if (bmi270_var.last_i2c_result==0) {
        return 0;
    }

    return (int16_t)(((uint16_t)temp_h << 8) | temp_l);
}

//=========================== private =========================================

void bmi270_delay(uint32_t cycles) {

    volatile uint32_t i;

    for (i=0;i<cycles;i++);
}

uint8_t bmi270_write_reg(uint8_t reg_addr, uint8_t value) {

    bmi270_var.last_i2c_result = i2c_write_bytes(reg_addr, &value, 1);

    return (bmi270_var.last_i2c_result!=0);
}

uint8_t bmi270_load_config_file(void) {

    uint16_t offset;
    uint16_t remaining;
    uint16_t chunk_len;
    uint8_t  init_ctrl;
    uint8_t  power_conf;
    uint8_t  chunk[BMI270_INIT_CONFIG_CHUNK_LEN];
    uint8_t  i;
    uint8_t  j;
    uint8_t  internal_status;

    power_conf = BMI270_PWR_CONF_PERF_MODE;
    if (bmi270_write_reg(BMI270_REG_ADDR_PWR_CONF, power_conf)==0) {
        return 0;
    }
    bmi270_delay(BMI270_DELAY_POLL_CYCLES);

    init_ctrl = 0x00;
    if (bmi270_write_reg(BMI270_REG_ADDR_INIT_CTRL, init_ctrl)==0) {
        return 0;
    }

    for (offset=0;offset<sizeof(bmi270_config_file);offset+=BMI270_INIT_CONFIG_CHUNK_LEN) {

        remaining = sizeof(bmi270_config_file) - offset;
        if (remaining>BMI270_INIT_CONFIG_CHUNK_LEN) {
            chunk_len = BMI270_INIT_CONFIG_CHUNK_LEN;
        } else {
            chunk_len = remaining;
        }

        if (bmi270_write_init_addr(offset)==0) {
            return 0;
        }

        for (j=0;j<chunk_len;j++) {
            chunk[j] = bmi270_config_file[offset+j];
        }

        bmi270_var.last_i2c_result = i2c_write_bytes(BMI270_REG_ADDR_INIT_DATA, chunk, chunk_len);
        if (bmi270_var.last_i2c_result==0) {
            return 0;
        }
    }

    init_ctrl = BMI270_INIT_CTRL_LOAD_DONE;
    if (bmi270_write_reg(BMI270_REG_ADDR_INIT_CTRL, init_ctrl)==0) {
        return 0;
    }

    bmi270_delay(BMI270_DELAY_CONFIG_LOAD_CYCLES);

    for (i=0;i<BMI270_INIT_STATUS_POLL_MAX;i++) {
        internal_status = bmi270_get_internal_status();
        if ((internal_status & 0x0f)==BMI270_INTERNAL_STATUS_INIT_OK) {
            return 1;
        }
        bmi270_delay(BMI270_DELAY_POLL_CYCLES);
    }

    return 0;
}

uint8_t bmi270_write_init_addr(uint16_t offset) {

    uint16_t asic_addr;
    uint8_t  init_addr[2];

    asic_addr = offset / 2;
    init_addr[0] = (uint8_t)(asic_addr & 0x0f);
    init_addr[1] = (uint8_t)((asic_addr >> 4) & 0xff);

    bmi270_var.last_i2c_result = i2c_write_bytes(BMI270_REG_ADDR_INIT_ADDR_0, init_addr, sizeof(init_addr));

    return (bmi270_var.last_i2c_result!=0);
}
