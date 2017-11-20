#ifndef STATICCONFIG_H
#define STATICCONFIG_H
#include "ch.h"
#include "hal.h"

#define VALID_MAGIC 0xADDA

struct configStructure {
	uint16_t samplingSpeed;
	uint8_t accelerometerRange;
	int16_t calibrationMPU[3];
	int16_t calibrationADXL[3];
	uint8_t calibrationDelay;
	uint8_t acquisitionDelay;
	bool gyroActivatedMPU;
	uint16_t magicNumber;
};

uint8_t restoreSystemConfigEEPROM(const SPIConfig *spiConfig,
				  struct configStructure *sysConfig);
void saveSystemConfigEEPROM(const SPIConfig *spiConfig,
			    struct configStructure *sysConfig);
void printSystemConfig(struct configStructure *sysConfig);

#endif /* STATICCONFIG_H */
