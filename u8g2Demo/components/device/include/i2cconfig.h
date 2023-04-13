#ifndef _I2CCONFIG_H_
#define _I2CCONFIG_H_

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define DATA_LENGTH 512   
#define I2C_MASTER_SCL_IO 22        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 100

#define I2C_SLAVE_NUM I2C_NUM_0    /*!<I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN  (2*DATA_LENGTH) /*!<I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN  (2*DATA_LENGTH) /*!<I2C slave rx buffer size */

esp_err_t i2c_master_init(void);
esp_err_t i2c_slave_init(void);
#endif