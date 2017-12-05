#ifndef EEPROM_H
#define EEPROM_H

#include "ch.h"
#include "hal.h"

int8_t	EEPROM_readBytes (SPIDriver *spip,
			   const SPIConfig *config,
			   uint8_t addr,
			   uint8_t length,
			   uint8_t *data);


bool EEPROM_writeBytes (SPIDriver *spip,
			const SPIConfig *config,
			uint8_t addr,
			uint8_t length,
			uint8_t *data);

#endif /* EEPROM_H */
