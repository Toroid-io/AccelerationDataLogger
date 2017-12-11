#ifndef STATICCONFIG_H
#define STATICCONFIG_H
#include "ch.h"
#include "hal.h"

#define VALID_MAGIC 0xADDA

struct configStructure {
	int16_t calibrationMPU[3];
	int16_t calibrationADXL[3];
	uint16_t magicNumber;
	uint16_t samplingSpeed;
	uint8_t accelerometerRange;
	uint8_t calibrationDelay;
	uint8_t acquisitionDelay;
	bool gyroActivatedMPU;
};


uint8_t checkSystemConfig(struct configStructure *sysConfig);
uint8_t restoreSystemConfigEEPROM(const SPIConfig *spiConfig,
				  struct configStructure *sysConfig);
void saveSystemConfigEEPROM(const SPIConfig *spiConfig,
			    struct configStructure *sysConfig);
void saveDefaultConfigEEPROM(const SPIConfig *spiConfig);
void printSystemConfig(struct configStructure *sysConfig, BaseSequentialStream *stream, bool simple);

#endif /* STATICCONFIG_H */
