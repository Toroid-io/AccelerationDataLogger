#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include <stdint.h>
#include <stdbool.h>

int8_t 	I2Cport_readBit (uint8_t devAddr,
		 uint8_t regAddr,
		 uint8_t bitNum,
		 uint8_t *data);
int8_t 	I2Cport_readBitW (uint8_t devAddr,
		  uint8_t regAddr,
		  uint8_t bitNum,
		  uint16_t *data);
int8_t 	I2Cport_readBits (uint8_t devAddr,
		  uint8_t regAddr, uint8_t bitStart,
		  uint8_t length,
		  uint8_t *data);
int8_t 	I2Cport_readBitsW (uint8_t devAddr,
		   uint8_t regAddr,
		   uint8_t bitStart,
		   uint8_t length,
		   uint16_t *data);
int8_t 	I2Cport_readByte (uint8_t devAddr,
		  uint8_t regAddr,
		  uint8_t *data);
int8_t 	I2Cport_readWord (uint8_t devAddr,
		  uint8_t regAddr,
		  uint16_t *data);
int8_t 	I2Cport_readBytes (uint8_t devAddr,
		   uint8_t regAddr,
		   uint8_t length,
		   uint8_t *data);
int8_t 	I2Cport_readWords (uint8_t devAddr,
		   uint8_t regAddr,
		   uint8_t length,
		   uint16_t *data);
bool	I2Cport_writeBit (uint8_t devAddr,
		  uint8_t regAddr,
		  uint8_t bitNum,
		  uint8_t data);
bool	I2Cport_writeBitW (uint8_t devAddr,
		   uint8_t regAddr,
		   uint8_t bitNum,
		   uint16_t data);
bool	I2Cport_writeBits (uint8_t devAddr,
		   uint8_t regAddr,
		   uint8_t bitStart,
		   uint8_t length,
		   uint8_t data);
bool	I2Cport_writeBitsW (uint8_t devAddr,
		    uint8_t regAddr,
		    uint8_t bitStart,
		    uint8_t length,
		    uint16_t data);
bool	I2Cport_writeByte (uint8_t devAddr,
		   uint8_t regAddr,
		   uint8_t data);
bool	I2Cport_writeWord (uint8_t devAddr,
		   uint8_t regAddr,
		   uint16_t data);
bool	I2Cport_writeBytes (uint8_t devAddr,
		    uint8_t regAddr,
		    uint8_t length,
		    uint8_t *data);
bool	I2Cport_writeWords (uint8_t devAddr,
		    uint8_t regAddr,
		    uint8_t length,
		    uint16_t *data);


#endif /* _I2CDEV_H_ */
