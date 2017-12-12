#include "ch.h"
#include "hal.h"
#include "ChibiOS/os/hal/lib/streams/chprintf.h"

#include "StaticSystemConfig.h"
#include "SPI_25LC01.h"

uint8_t checkSystemConfig(struct configStructure *sysConfig)
{
	if (sysConfig->magicNumber == VALID_MAGIC)
		return 0;
	else
		return 1;
}

uint8_t restoreSystemConfigEEPROM(const SPIConfig *spiConfig,
				  struct configStructure *sysConfig)
{
	EEPROM_readBytes(&SPID1,
			 spiConfig,
			 0,
			 sizeof(struct configStructure),
			 (uint8_t *)sysConfig);
	return checkSystemConfig(sysConfig);
}

void saveSystemConfigEEPROM(const SPIConfig *spiConfig,
			    struct configStructure *sysConfig)
{
	EEPROM_writeBytes(&SPID1,
			  spiConfig,
			  0,
			  sizeof(struct configStructure),
			  (uint8_t *)sysConfig);
}

void saveDefaultConfigEEPROM(const SPIConfig *spiConfig)
{
	struct configStructure defaultConfig;
	defaultConfig.samplingSpeed = 800;
	defaultConfig.accelerometerRange = 2;
	defaultConfig.calibrationMPU[0] = 0;
	defaultConfig.calibrationMPU[1] = 0;
	defaultConfig.calibrationMPU[2] = 0;
	defaultConfig.calibrationADXL[0] = 0;
	defaultConfig.calibrationADXL[1] = 0;
	defaultConfig.calibrationADXL[2] = 0;
	defaultConfig.gyroActivatedMPU = false;
	defaultConfig.calibrationDelay = 3;
	defaultConfig.acquisitionDelay = 3;
	defaultConfig.magicNumber = VALID_MAGIC;
	defaultConfig.filterType = 0;
	EEPROM_writeBytes(&SPID1,
			  spiConfig,
			  0,
			  sizeof(struct configStructure),
			  (uint8_t *)&defaultConfig);
}
void printSystemConfig(struct configStructure *sysConfig, BaseSequentialStream *stream, bool simple)
{
	if (simple) {
		chprintf(stream, "0x%x\r\n", sysConfig->magicNumber);
		chprintf(stream, "%u\r\n", sysConfig->samplingSpeed);
		chprintf(stream, "%u\r\n",
			 (uint16_t)sysConfig->accelerometerRange);
		chprintf(stream, "%d %d %d\r\n",
			 sysConfig->calibrationMPU[0],
			 sysConfig->calibrationMPU[1],
			 sysConfig->calibrationMPU[2]);
		chprintf(stream, "%d %d %d\r\n",
			 sysConfig->calibrationADXL[0],
			 sysConfig->calibrationADXL[1],
			 sysConfig->calibrationADXL[2]);
		chprintf(stream, "%u\r\n",
			 (uint16_t)sysConfig->calibrationDelay);
		chprintf(stream, "%u\r\n",
			 (uint16_t)sysConfig->acquisitionDelay);

	} else {
		chprintf(stream, "- - - - - - - - - - - - - - - - - - - -\r\n");
		chprintf(stream, "Sampling speed: %u\r\n", sysConfig->samplingSpeed);
		chprintf(stream, "Accel range: %u\r\n",
			 (uint16_t)sysConfig->accelerometerRange);
		chprintf(stream, "Calibration MPU: %d %d %d\r\n",
			 sysConfig->calibrationMPU[0],
			 sysConfig->calibrationMPU[1],
			 sysConfig->calibrationMPU[2]);
		chprintf(stream, "Calibration ADXL: %d %d %d\r\n",
			 sysConfig->calibrationADXL[0],
			 sysConfig->calibrationADXL[1],
			 sysConfig->calibrationADXL[2]);
		chprintf(stream, "Calibration delay: %u\r\n",
			 (uint16_t)sysConfig->calibrationDelay);
		chprintf(stream, "Acquisition delay: %u\r\n",
			 (uint16_t)sysConfig->acquisitionDelay);
		chprintf(stream, "Gyro MPU Activated: %u\r\n",
			 (uint16_t)sysConfig->gyroActivatedMPU);
		chprintf(stream, "Magic: 0x%x\r\n", sysConfig->magicNumber);
		chprintf(stream, "- - - - - - - - - - - - - - - - - - - -\r\n");

	}
}

