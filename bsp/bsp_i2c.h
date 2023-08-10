#ifndef BSP_I2C_H
#define BSP_I2C_H
#include "struct_typedef.h"
#include "main.h"

#define I2C_ACK 1
#define I2C_NO_ACK  0

extern void bsp_I2C_master_transmit(I2C_TypeDef *I2C, uint16_t I2C_address, uint8_t *data, uint16_t len);
extern void bsp_I2C_master_receive(I2C_TypeDef *I2C, uint16_t I2C_address, uint8_t *data, uint16_t len);

#endif
