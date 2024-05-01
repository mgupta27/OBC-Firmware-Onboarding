#include "lm75bd.h"
#include "i2c_io.h"
#include "errors.h"
#include "logging.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

/* LM75BD Registers (p.8) */
#define LM75BD_REG_TEMP 0x00U  /* Temperature Register */
#define LM75BD_REG_CONF 0x01U  /* Configuration Register (R/W) */

/* Temperature Conversion Constants */
#define TEMP_CONVERSION_VAL 0.125f  /* Digital to Analog conversion rate */
#define ELEVEN_BIT_MASK 0x07FFU     /* Mask to take first 11 bits from lsb side -> 0000 0111 1111 1111 */

error_code_t lm75bdInit(lm75bd_config_t *config) {
  error_code_t errCode;

  if (config == NULL) return ERR_CODE_INVALID_ARG;

  RETURN_IF_ERROR_CODE(writeConfigLM75BD(config->devAddr, config->osFaultQueueSize, config->osPolarity,
                                         config->osOperationMode, config->devOperationMode));

  // Assume that the overtemperature and hysteresis thresholds are already set
  // Hysteresis: 75 degrees Celsius
  // Overtemperature: 80 degrees Celsius

  return ERR_CODE_SUCCESS;
}

#define TEMP_WRITE_BUFF_SIZE 1U
#define TEMP_READ_BUFF_SIZE 2U
#define TEMP_CONVERSION_VAL 0.125f
#define ELEVEN_BIT_MASK 0x07FFU
error_code_t readTempLM75BD(uint8_t devAddr, float *temp) {
  error_code_t errCode;

  // Stores the register address to be written
  uint8_t writeBuff = LM75BD_REG_TEMP;

  // Stores the data read from the sensor (first byte is stored in index 0)
  // 0: Most Significant Byte (MSB)
  // 1: Least SignificantByte (LSB)
  uint8_t readBuff[TEMP_READ_BUFF_SIZE] = {0};
  
  // Stores the data read from the temperature register 
  uint16_t tempRegValue = 0x00U;

  // Stores if the temperature is positive or negative
  uint8_t isTempPositive;

  errCode = i2cSendTo(LM75BD_OBC_I2C_ADDR, &writeBuff, TEMP_WRITE_BUFF_SIZE);
  if (errCode != ERR_CODE_SUCCESS) return errCode;

  errCode = i2cReceiveFrom(LM75BD_OBC_I2C_ADDR, readBuff, TEMP_READ_BUFF_SIZE);
  if (errCode != ERR_CODE_SUCCESS) return errCode;

  // Assemble 16-bit value
  tempRegValue |= (readBuff[0] << 8);
  tempRegValue |= readBuff[1];

  // Poll D10 (15th bit) to determine if temperature value is positive or negative
  // D10 = 0: Temperature is positive
  // D10 = 1: Temperature is negative
  isTempPositive = !((tempRegValue >> 15) & 0x01);

  // Store the 11-bit value by removing the 5 unused bits
  tempRegValue = tempRegValue >> 5;

  // Calculate and store temperature value
  if (isTempPositive) {
    *temp = tempRegValue * TEMP_CONVERSION_VAL;
  } else {
    tempRegValue = (~tempRegValue) + 0x01;  // Perform 2's complement
    tempRegValue &= ELEVEN_BIT_MASK;        // Take the first 11 bits starting from the lsb
    *temp = -1 * tempRegValue * TEMP_CONVERSION_VAL;
  }

  return ERR_CODE_SUCCESS;
}

#define CONF_WRITE_BUFF_SIZE 2U
error_code_t writeConfigLM75BD(uint8_t devAddr, uint8_t osFaultQueueSize, uint8_t osPolarity,
                                   uint8_t osOperationMode, uint8_t devOperationMode) {
  error_code_t errCode;

  // Stores the register address and data to be written
  // 0: Register address
  // 1: Data
  uint8_t buff[CONF_WRITE_BUFF_SIZE] = {0};

  buff[0] = LM75BD_REG_CONF;

  uint8_t osFaltQueueRegData = 0;
  switch (osFaultQueueSize) {
    case 1:
      osFaltQueueRegData = 0;
      break;
    case 2:
      osFaltQueueRegData = 1;
      break;
    case 4:
      osFaltQueueRegData = 2;
      break;
    case 6:
      osFaltQueueRegData = 3;
      break;
    default:
      return ERR_CODE_INVALID_ARG;
  }

  buff[1] |= (osFaltQueueRegData << 3);
  buff[1] |= (osPolarity << 2);
  buff[1] |= (osOperationMode << 1);
  buff[1] |= devOperationMode;

  errCode = i2cSendTo(LM75BD_OBC_I2C_ADDR, buff, CONF_WRITE_BUFF_SIZE);
  if (errCode != ERR_CODE_SUCCESS) return errCode;

  return ERR_CODE_SUCCESS;
}
