#include "ch.h"
#include "hal.h"
#include "I2Cdev.h"

int8_t 	I2Cport_readBit (I2CDriver *i2cp,
			 uint8_t devAddr,
			 uint8_t regAddr,
			 uint8_t bitNum,
			 uint8_t *data)
{
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(4);

	uint8_t b;
	uint8_t tx_data[8];
	tx_data[0] = regAddr;

	/* get register value */
	i2cAcquireBus(i2cp);
	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 1,
					  &b, 1, tmo);

	i2cflags_t flags = 0;
	switch(status) {
	case MSG_OK:
		break;
	case MSG_RESET:
		flags = i2cGetErrors(i2cp);
		osalDbgCheck(MSG_OK == status);
		break;
	case MSG_TIMEOUT:
		flags = i2cGetErrors(i2cp);
		osalDbgCheck(MSG_OK == status);
		break;
	}
	osalDbgCheck(MSG_OK == status);
	if (status != MSG_OK)
		return false;

	*data = (b >> bitNum) & 0x01;

	i2cReleaseBus(i2cp);
	osalDbgCheck(MSG_OK == status);

	return (MSG_OK == status) ? true : false;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cport_readBits(I2CDriver *i2cp,
			uint8_t devAddr,
			uint8_t regAddr,
			uint8_t bitStart,
			uint8_t length,
			uint8_t *data)
{
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(4);

	uint8_t b;
	uint8_t tx_data[8];
	tx_data[0] = regAddr;

	/* receiving */
	i2cAcquireBus(i2cp);
	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 1,
					  &b, 1, tmo);
	i2cReleaseBus(i2cp);

	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	b &= mask;
	b >>= (bitStart - length + 1);
	*data = b;

	osalDbgCheck(MSG_OK == status);
	return (MSG_OK == status) ? length : -1;
}

int8_t	I2Cport_readBytes (I2CDriver *i2cp,
			   uint8_t devAddr,
			   uint8_t regAddr,
			   uint8_t length,
			   uint8_t *data)
{
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(4);

	uint8_t tx_data[8];
	tx_data[0] = regAddr;

	/* receiving */
	i2cAcquireBus(i2cp);
	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 1,
					  data, length, tmo);
	i2cReleaseBus(i2cp);

	osalDbgCheck(MSG_OK == status);
	return (MSG_OK == status) ? length : -1;
}

int8_t	I2Cport_readByte (I2CDriver *i2cp,
			  uint8_t devAddr,
			  uint8_t regAddr,
			  uint8_t *data)
{
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(4);

	uint8_t tx_data[8];
	tx_data[0] = regAddr;

	/* receiving */
	i2cAcquireBus(i2cp);
	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 1,
					  data, 1, tmo);
	i2cReleaseBus(i2cp);

	osalDbgCheck(MSG_OK == status);
	return (MSG_OK == status) ? 1 : -1;
}



/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool	I2Cport_writeBit (I2CDriver *i2cp,
			  uint8_t devAddr,
			  uint8_t regAddr,
			  uint8_t bitNum,
			  uint8_t data)
{
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(4);

	uint8_t b;
	uint8_t tx_data[8];
	tx_data[0] = regAddr;

	/* get register value */
	i2cAcquireBus(i2cp);
	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 1,
					  &b, 1, tmo);

	i2cflags_t flags = 0;
	switch(status) {
	case MSG_OK:
		break;
	case MSG_RESET:
		flags = i2cGetErrors(i2cp);
		osalDbgCheck(MSG_OK == status);
		break;
	case MSG_TIMEOUT:
		flags = i2cGetErrors(i2cp);
		osalDbgCheck(MSG_OK == status);
		break;
	}
	osalDbgCheck(MSG_OK == status);
	if (status != MSG_OK)
		return false;

	/* write the corresponding bit in temp variable */
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	tx_data[0] = regAddr;
	tx_data[1] = b;

	/* send the new register value */
	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 2,
					  NULL, 0, tmo);
	i2cReleaseBus(i2cp);
	osalDbgCheck(MSG_OK == status);

	return (MSG_OK == status) ? true : false;
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool	I2Cport_writeBits (I2CDriver *i2cp,
			   uint8_t devAddr,
			   uint8_t regAddr,
			   uint8_t bitStart,
			   uint8_t length,
			   uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(4);

	uint8_t b;
	uint8_t tx_data[8];
	tx_data[0] = regAddr;

	/* get register value */
	i2cAcquireBus(i2cp);
	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 1,
					  &b, 1, tmo);

	osalDbgCheck(MSG_OK == status);
	if (status != MSG_OK)
		return false;

	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	b &= ~(mask); // zero all important bits in existing byte
	b |= data; // combine data with existing byte

	tx_data[0] = regAddr;
	tx_data[1] = b;

	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 2,
					  NULL, 0, tmo);
	i2cReleaseBus(i2cp);
	osalDbgCheck(MSG_OK == status);

	return (MSG_OK == status) ? true : false;
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cport_writeByte(I2CDriver *i2cp,
		       uint8_t devAddr,
		       uint8_t regAddr,
		       uint8_t data) {
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(4);
	//systime_t tmo = TIME_INFINITE;

	i2cAcquireBus(i2cp);
	uint8_t tx_data[8];
	tx_data[0] = regAddr;
	tx_data[1] = data;

	/* send the new register value */
	status = i2cMasterTransmitTimeout(i2cp, devAddr,
					  tx_data, 2,
					  NULL, 0, tmo);
	i2cReleaseBus(i2cp);
	i2cflags_t flags = 0;
	switch(status) {
	case MSG_OK:
		break;
	case MSG_RESET:
		flags = i2cGetErrors(i2cp);
		osalDbgCheck(MSG_OK == status);
		break;
	case MSG_TIMEOUT:
		flags = i2cGetErrors(i2cp);
		osalDbgCheck(MSG_OK == status);
		break;
	}

	return (MSG_OK == status) ? true : false;
}
