/**
 * @file ioex.h
 * @author Shahid Hashmi (shahidh@acevin.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* General */
#define I2C "/dev/i2c"
#define SLAVE_ADDR

/* Command */
#define INPUT_PORT_0    0x00
#define INPUT_PORT_1    0x01
#define OUTPUT_PORT_0   0x02
#define OUTPUT_PORT_1   0x03
#define POLARITY_INVERSION_PORT_0   0x04
#define POLARITY_INVERSION_PORT_1   0x05
#define CONFIGURATION_PORT_0    0x06
#define CONFIGURATION_PORT_1    0x07

/* Functions */
sti2c_t *ioex_init(sti2c_t I2C_dev, uint8_t addr, uint8_t *pdata);
void ioex_deinit(sti2c_t I2C_dev);

