#include "ch.h"
#include "hal.h"
#include "EEPROM.h"

int8_t	EEPROM_readBytes (SPIDriver *spip,
			   const SPIConfig *config,
			   uint8_t addr,
			   uint8_t length,
			   uint8_t *data)
{

	char writeCommand[2];

	writeCommand[0] = 0x3;
	writeCommand[1] = addr & 0x7F;

	/* receiving */
	spiAcquireBus(spip);
	spiStart(spip, config);
	spiSelect(spip);
	spiSend(spip, 2, writeCommand);
	spiReceive(spip, length, data);
	spiUnselect(spip);
	spiReleaseBus(spip);

	return 0;
}

bool EEPROM_writeBytes (SPIDriver *spip,
			const SPIConfig *config,
			uint8_t addr,
			uint8_t length,
			uint8_t *data)
{
	char writeCommand[2];
	char wren = 0x6;

	writeCommand[0] = 0x2;

	/* This device can write data in 16 bytes (page size) chunks */
	uint8_t npages = length/16 + 1;

	/* Write n pages in the loop */
	for (uint8_t i = 0; i < npages - 1; ++i){
		writeCommand[1] = (addr + i*16) & 0x7F;
		spiAcquireBus(spip);
		spiStart(spip, config);
		spiSelect(spip);
		spiSend(spip, 1, &wren);
		spiUnselect(spip);
		/* Bring CS up and wait to set WREN latch */
		chThdSleepMilliseconds(1);
		spiSelect(spip);
		spiSend(spip, 2, writeCommand);
		spiSend(spip, 16, data + i*16);
		spiUnselect(spip);
		spiReleaseBus(spip);
		/* Wait until the page is written */
		chThdSleepMilliseconds(6);
	}

	/* Write the remaining bytes */
	writeCommand[1] = (addr + (npages - 1)*16) & 0x7F;
	spiAcquireBus(spip);
	spiStart(spip, config);
	spiSelect(spip);
	spiSend(spip, 1, &wren);
	spiUnselect(spip);
	/* Bring CS up and wait to set WREN latch */
	chThdSleepMilliseconds(1);
	spiSelect(spip);
	spiSend(spip, 2, writeCommand);
	spiSend(spip, length%16, data + (npages -1)*16);
	spiUnselect(spip);
	spiReleaseBus(spip);
	/* Wait until the page is written */
	chThdSleepMilliseconds(6);

	return 0;
}
