#ifndef STATICCONFIG_H
#define STATICCONFIG_H
#include "ch.h"
#include "hal.h"

#define VALID_MAGIC 0xADDA

struct configStructure {
	volatile int16_t calibrationMPU[3];
	volatile int16_t calibrationADXL[3];
	uint16_t magicNumber;
	volatile uint16_t samplingSpeed;
	volatile uint8_t accelerometerRange;
	volatile uint8_t calibrationDelay;
	volatile uint8_t acquisitionDelay;
	volatile uint8_t MPUfilterType;
	volatile uint8_t ADXLfilterType;
	volatile bool gyroActivatedMPU;
};


uint8_t checkSystemConfig(struct configStructure *sysConfig);
uint8_t restoreSystemConfigEEPROM(const SPIConfig *spiConfig,
				  struct configStructure *sysConfig);
void saveSystemConfigEEPROM(const SPIConfig *spiConfig,
			    struct configStructure *sysConfig);
void saveDefaultConfigEEPROM(const SPIConfig *spiConfig);
void printSystemConfig(struct configStructure *sysConfig,
		       BaseSequentialStream *stream);

#endif /* STATICCONFIG_H */
