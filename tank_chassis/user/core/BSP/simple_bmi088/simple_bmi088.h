#ifndef USER_CORE_BSP_SIMPLE_BMI088_H
#define USER_CORE_BSP_SIMPLE_BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

typedef enum
{
  BMI088_INIT_STAGE_NOT_STARTED = 0U,
  BMI088_INIT_STAGE_CHECK_CHIP_IDS,
  BMI088_INIT_STAGE_SOFT_RESET,
  BMI088_INIT_STAGE_RESTART_SPI,
  BMI088_INIT_STAGE_CHECK_CHIP_IDS_AFTER_RESET,
  BMI088_INIT_STAGE_CONFIGURE,
  BMI088_INIT_STAGE_INITIAL_READ,
  BMI088_INIT_STAGE_READY
} BMI088_InitStage;

typedef struct
{
  float accel_mps2[3];
  float gyro_dps[3];
  float angular_accel_dps2[3];
  float gyro_bias_dps[3];
  float euler_deg[3];
  float quaternion[4];
  float accel_norm_mps2;
  uint8_t accel_chip_id;
  uint8_t gyro_chip_id;
  uint8_t initialized;
  uint8_t euler_valid;
  uint8_t gyro_calibrated;
  uint8_t init_stage;
  uint8_t init_status;
  uint8_t accel_spi_status;
  uint8_t gyro_spi_status;
  uint32_t update_count;
  uint32_t error_count;
} BMI088_Data;

extern volatile BMI088_Data g_bmi088;

HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef BMI088_Read(void);

#ifdef __cplusplus
}
#endif

#endif
