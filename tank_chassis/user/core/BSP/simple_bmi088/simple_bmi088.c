#include "simple_bmi088.h"
#include "cmsis_os2.h"
#include <math.h>

#define BMI088_ACCEL_CS_PORT GPIOC
#define BMI088_ACCEL_CS_PIN GPIO_PIN_0
#define BMI088_GYRO_CS_PORT GPIOC
#define BMI088_GYRO_CS_PIN GPIO_PIN_3

#define BMI088_ACCEL_CHIP_ID_REG 0x00U
#define BMI088_ACCEL_CHIP_ID 0x1EU
#define BMI088_ACCEL_DATA_REG 0x12U
#define BMI088_ACCEL_CONF_REG 0x40U
#define BMI088_ACCEL_RANGE_REG 0x41U
#define BMI088_ACCEL_PWR_CONF_REG 0x7CU
#define BMI088_ACCEL_PWR_CTRL_REG 0x7DU
#define BMI088_ACCEL_SOFTRESET_REG 0x7EU
#define BMI088_ACCEL_INT1_IO_CTRL_REG 0x53U
#define BMI088_ACCEL_INT_MAP_DATA_REG 0x58U

#define BMI088_GYRO_CHIP_ID_REG 0x00U
#define BMI088_GYRO_CHIP_ID 0x0FU
#define BMI088_GYRO_DATA_REG 0x02U
#define BMI088_GYRO_RANGE_REG 0x0FU
#define BMI088_GYRO_BANDWIDTH_REG 0x10U
#define BMI088_GYRO_SOFTRESET_REG 0x14U
#define BMI088_GYRO_INT_CTRL_REG 0x15U
#define BMI088_GYRO_INT3_INT4_IO_CONF_REG 0x16U
#define BMI088_GYRO_INT3_INT4_IO_MAP_REG 0x18U

#define BMI088_SPI_TIMEOUT_MS 5U
#define BMI088_GRAVITY_MPS2 9.80665f
#define BMI088_RAD_TO_DEG 57.2957795131f
#define BMI088_DEG_TO_RAD 0.0174532925199f
#define BMI088_MAHONY_KP 1.0f
#define BMI088_ANGULAR_ACCEL_FILTER_ALPHA 0.2f
#define BMI088_GYRO_CALIBRATION_SAMPLES 500U

volatile BMI088_Data g_bmi088 = {0};

static SPI_HandleTypeDef *bmi088_hspi;
static uint32_t bmi088_last_cycle;
static uint32_t bmi088_cpu_hz;
static uint32_t bmi088_gyro_calibration_count;
static float bmi088_gyro_calibration_sum[3];
static float bmi088_previous_gyro_dps[3];
static uint8_t bmi088_previous_gyro_valid;

static void BMI088_DelayMs(uint32_t delay_ms)
{
  if (osKernelGetState() == osKernelRunning)
  {
    (void)osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static void BMI088_RecordSpiStatus(GPIO_TypeDef *port, uint16_t pin, HAL_StatusTypeDef status)
{
  if ((port == BMI088_ACCEL_CS_PORT) && (pin == BMI088_ACCEL_CS_PIN))
  {
    g_bmi088.accel_spi_status = (uint8_t)status;
  }
  else if ((port == BMI088_GYRO_CS_PORT) && (pin == BMI088_GYRO_CS_PIN))
  {
    g_bmi088.gyro_spi_status = (uint8_t)status;
  }
}

static void BMI088_Select(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void BMI088_Deselect(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef BMI088_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};
  HAL_StatusTypeDef status;

  BMI088_Select(port, pin);
  status = HAL_SPI_Transmit(bmi088_hspi, tx, sizeof(tx), BMI088_SPI_TIMEOUT_MS);
  BMI088_Deselect(port, pin);
  BMI088_RecordSpiStatus(port, pin, status);
  return status;
}

static HAL_StatusTypeDef BMI088_AccelRead(uint8_t reg, uint8_t *data, uint16_t length)
{
  uint8_t tx[8] = {0};
  uint8_t rx[8] = {0};
  HAL_StatusTypeDef status;

  if (length > 6U)
  {
    return HAL_ERROR;
  }

  tx[0] = reg | 0x80U;
  BMI088_Select(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN);
  status = HAL_SPI_TransmitReceive(bmi088_hspi, tx, rx, length + 2U, BMI088_SPI_TIMEOUT_MS);
  BMI088_Deselect(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN);
  g_bmi088.accel_spi_status = (uint8_t)status;

  if (status == HAL_OK)
  {
    for (uint16_t i = 0U; i < length; ++i)
    {
      data[i] = rx[i + 2U];
    }
  }
  return status;
}

static HAL_StatusTypeDef BMI088_GyroRead(uint8_t reg, uint8_t *data, uint16_t length)
{
  uint8_t tx[7] = {0};
  uint8_t rx[7] = {0};
  HAL_StatusTypeDef status;

  if (length > 6U)
  {
    return HAL_ERROR;
  }

  tx[0] = reg | 0x80U;
  BMI088_Select(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);
  status = HAL_SPI_TransmitReceive(bmi088_hspi, tx, rx, length + 1U, BMI088_SPI_TIMEOUT_MS);
  BMI088_Deselect(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);
  g_bmi088.gyro_spi_status = (uint8_t)status;

  if (status == HAL_OK)
  {
    for (uint16_t i = 0U; i < length; ++i)
    {
      data[i] = rx[i + 1U];
    }
  }
  return status;
}

static int16_t BMI088_ToInt16(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static float BMI088_Clamp(float value, float lower, float upper)
{
  if (value < lower)
  {
    return lower;
  }
  if (value > upper)
  {
    return upper;
  }
  return value;
}

static void BMI088_InitializeQuaternionFromAccel(float ax, float ay, float az)
{
  const float roll = atan2f(ay, az);
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float half_roll = 0.5f * roll;
  const float half_pitch = 0.5f * pitch;
  const float cos_roll = cosf(half_roll);
  const float sin_roll = sinf(half_roll);
  const float cos_pitch = cosf(half_pitch);
  const float sin_pitch = sinf(half_pitch);

  g_bmi088.quaternion[0] = cos_roll * cos_pitch;
  g_bmi088.quaternion[1] = sin_roll * cos_pitch;
  g_bmi088.quaternion[2] = cos_roll * sin_pitch;
  g_bmi088.quaternion[3] = -sin_roll * sin_pitch;
  g_bmi088.euler_valid = 1U;
}

static void BMI088_UpdateAttitude(void)
{
  const float ax = g_bmi088.accel_mps2[0];
  const float ay = g_bmi088.accel_mps2[1];
  const float az = g_bmi088.accel_mps2[2];
  const uint32_t now_cycle = DWT->CYCCNT;
  const uint32_t elapsed_cycles = now_cycle - bmi088_last_cycle;
  float gyro_dps[3];
  float dt_s;

  bmi088_last_cycle = now_cycle;
  dt_s = (bmi088_cpu_hz == 0U) ? 0.001f : (float)elapsed_cycles / (float)bmi088_cpu_hz;
  if ((dt_s <= 0.0f) || (dt_s > 0.02f))
  {
    dt_s = 0.001f;
  }

  g_bmi088.accel_norm_mps2 = sqrtf(ax * ax + ay * ay + az * az);
  if ((g_bmi088.accel_norm_mps2 > (0.85f * BMI088_GRAVITY_MPS2)) &&
      (g_bmi088.accel_norm_mps2 < (1.15f * BMI088_GRAVITY_MPS2)) &&
      (g_bmi088.gyro_calibrated == 0U))
  {
    const float gyro_abs_sum = ((g_bmi088.gyro_dps[0] < 0.0f) ? -g_bmi088.gyro_dps[0] : g_bmi088.gyro_dps[0]) +
                               ((g_bmi088.gyro_dps[1] < 0.0f) ? -g_bmi088.gyro_dps[1] : g_bmi088.gyro_dps[1]) +
                               ((g_bmi088.gyro_dps[2] < 0.0f) ? -g_bmi088.gyro_dps[2] : g_bmi088.gyro_dps[2]);

    if (gyro_abs_sum < 5.0f)
    {
      for (uint32_t i = 0U; i < 3U; ++i)
      {
        bmi088_gyro_calibration_sum[i] += g_bmi088.gyro_dps[i];
      }
      bmi088_gyro_calibration_count++;
      if (bmi088_gyro_calibration_count >= BMI088_GYRO_CALIBRATION_SAMPLES)
      {
        for (uint32_t i = 0U; i < 3U; ++i)
        {
          g_bmi088.gyro_bias_dps[i] = bmi088_gyro_calibration_sum[i] / (float)bmi088_gyro_calibration_count;
        }
        g_bmi088.gyro_calibrated = 1U;
      }
    }
    else
    {
      bmi088_gyro_calibration_count = 0U;
      bmi088_gyro_calibration_sum[0] = 0.0f;
      bmi088_gyro_calibration_sum[1] = 0.0f;
      bmi088_gyro_calibration_sum[2] = 0.0f;
    }
  }

  for (uint32_t i = 0U; i < 3U; ++i)
  {
    gyro_dps[i] = g_bmi088.gyro_dps[i] - g_bmi088.gyro_bias_dps[i];
    if (bmi088_previous_gyro_valid != 0U)
    {
      const float raw_angular_accel = (gyro_dps[i] - bmi088_previous_gyro_dps[i]) / dt_s;
      g_bmi088.angular_accel_dps2[i] = BMI088_ANGULAR_ACCEL_FILTER_ALPHA * raw_angular_accel +
                                       (1.0f - BMI088_ANGULAR_ACCEL_FILTER_ALPHA) * g_bmi088.angular_accel_dps2[i];
    }
    bmi088_previous_gyro_dps[i] = gyro_dps[i];
  }
  bmi088_previous_gyro_valid = 1U;

  if ((g_bmi088.accel_norm_mps2 > (0.85f * BMI088_GRAVITY_MPS2)) &&
      (g_bmi088.accel_norm_mps2 < (1.15f * BMI088_GRAVITY_MPS2)))
  {
    if (g_bmi088.euler_valid == 0U)
    {
      BMI088_InitializeQuaternionFromAccel(ax, ay, az);
    }

    if (g_bmi088.euler_valid != 0U)
    {
      const float inv_accel_norm = 1.0f / g_bmi088.accel_norm_mps2;
      const float normalized_ax = ax * inv_accel_norm;
      const float normalized_ay = ay * inv_accel_norm;
      const float normalized_az = az * inv_accel_norm;
      const float q0 = g_bmi088.quaternion[0];
      const float q1 = g_bmi088.quaternion[1];
      const float q2 = g_bmi088.quaternion[2];
      const float q3 = g_bmi088.quaternion[3];
      const float estimated_gx = 2.0f * (q1 * q3 - q0 * q2);
      const float estimated_gy = 2.0f * (q0 * q1 + q2 * q3);
      const float estimated_gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

      gyro_dps[0] += BMI088_MAHONY_KP * (normalized_ay * estimated_gz - normalized_az * estimated_gy) * BMI088_RAD_TO_DEG;
      gyro_dps[1] += BMI088_MAHONY_KP * (normalized_az * estimated_gx - normalized_ax * estimated_gz) * BMI088_RAD_TO_DEG;
      gyro_dps[2] += BMI088_MAHONY_KP * (normalized_ax * estimated_gy - normalized_ay * estimated_gx) * BMI088_RAD_TO_DEG;
    }
  }

  if (g_bmi088.euler_valid != 0U)
  {
    const float q0 = g_bmi088.quaternion[0];
    const float q1 = g_bmi088.quaternion[1];
    const float q2 = g_bmi088.quaternion[2];
    const float q3 = g_bmi088.quaternion[3];
    const float gx_rad = gyro_dps[0] * BMI088_DEG_TO_RAD;
    const float gy_rad = gyro_dps[1] * BMI088_DEG_TO_RAD;
    const float gz_rad = gyro_dps[2] * BMI088_DEG_TO_RAD;
    float norm;

    g_bmi088.quaternion[0] += 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) * dt_s;
    g_bmi088.quaternion[1] += 0.5f * (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) * dt_s;
    g_bmi088.quaternion[2] += 0.5f * (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) * dt_s;
    g_bmi088.quaternion[3] += 0.5f * (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) * dt_s;

    norm = sqrtf(g_bmi088.quaternion[0] * g_bmi088.quaternion[0] +
                 g_bmi088.quaternion[1] * g_bmi088.quaternion[1] +
                 g_bmi088.quaternion[2] * g_bmi088.quaternion[2] +
                 g_bmi088.quaternion[3] * g_bmi088.quaternion[3]);
    if (norm > 0.0f)
    {
      const float inv_norm = 1.0f / norm;
      const float normalized_q0 = g_bmi088.quaternion[0] * inv_norm;
      const float normalized_q1 = g_bmi088.quaternion[1] * inv_norm;
      const float normalized_q2 = g_bmi088.quaternion[2] * inv_norm;
      const float normalized_q3 = g_bmi088.quaternion[3] * inv_norm;

      g_bmi088.quaternion[0] = normalized_q0;
      g_bmi088.quaternion[1] = normalized_q1;
      g_bmi088.quaternion[2] = normalized_q2;
      g_bmi088.quaternion[3] = normalized_q3;
      g_bmi088.euler_deg[0] = atan2f(2.0f * (normalized_q0 * normalized_q1 + normalized_q2 * normalized_q3),
                                      1.0f - 2.0f * (normalized_q1 * normalized_q1 + normalized_q2 * normalized_q2)) * BMI088_RAD_TO_DEG;
      g_bmi088.euler_deg[1] = asinf(BMI088_Clamp(2.0f * (normalized_q0 * normalized_q2 - normalized_q3 * normalized_q1), -1.0f, 1.0f)) * BMI088_RAD_TO_DEG;
      g_bmi088.euler_deg[2] = atan2f(2.0f * (normalized_q0 * normalized_q3 + normalized_q1 * normalized_q2),
                                      1.0f - 2.0f * (normalized_q2 * normalized_q2 + normalized_q3 * normalized_q3)) * BMI088_RAD_TO_DEG;
    }
  }
}

static HAL_StatusTypeDef BMI088_CheckChipIds(void)
{
  HAL_StatusTypeDef accel_status;
  HAL_StatusTypeDef gyro_status;

  accel_status = BMI088_AccelRead(BMI088_ACCEL_CHIP_ID_REG, (uint8_t *)&g_bmi088.accel_chip_id, 1U);
  gyro_status = BMI088_GyroRead(BMI088_GYRO_CHIP_ID_REG, (uint8_t *)&g_bmi088.gyro_chip_id, 1U);

  if ((accel_status != HAL_OK) || (gyro_status != HAL_OK) ||
      (g_bmi088.accel_chip_id != BMI088_ACCEL_CHIP_ID) ||
      (g_bmi088.gyro_chip_id != BMI088_GYRO_CHIP_ID))
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef status;
  uint8_t discard;

  if (hspi == NULL)
  {
    g_bmi088.init_status = (uint8_t)HAL_ERROR;
    return HAL_ERROR;
  }

  bmi088_hspi = hspi;
  g_bmi088.initialized = 0U;
  g_bmi088.init_stage = BMI088_INIT_STAGE_CHECK_CHIP_IDS;
  g_bmi088.init_status = (uint8_t)HAL_ERROR;
  g_bmi088.euler_valid = 0U;
  g_bmi088.gyro_calibrated = 0U;
  bmi088_previous_gyro_valid = 0U;
  g_bmi088.euler_deg[0] = 0.0f;
  g_bmi088.euler_deg[1] = 0.0f;
  g_bmi088.euler_deg[2] = 0.0f;
  g_bmi088.quaternion[0] = 1.0f;
  g_bmi088.quaternion[1] = 0.0f;
  g_bmi088.quaternion[2] = 0.0f;
  g_bmi088.quaternion[3] = 0.0f;
  g_bmi088.angular_accel_dps2[0] = 0.0f;
  g_bmi088.angular_accel_dps2[1] = 0.0f;
  g_bmi088.angular_accel_dps2[2] = 0.0f;
  bmi088_gyro_calibration_count = 0U;
  bmi088_gyro_calibration_sum[0] = 0.0f;
  bmi088_gyro_calibration_sum[1] = 0.0f;
  bmi088_gyro_calibration_sum[2] = 0.0f;
  bmi088_previous_gyro_dps[0] = 0.0f;
  bmi088_previous_gyro_dps[1] = 0.0f;
  bmi088_previous_gyro_dps[2] = 0.0f;
  BMI088_Deselect(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN);
  BMI088_Deselect(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);

  status = BMI088_CheckChipIds();
  if (status != HAL_OK)
  {
    g_bmi088.error_count++;
    g_bmi088.init_status = (uint8_t)status;
    return status;
  }

  g_bmi088.init_stage = BMI088_INIT_STAGE_SOFT_RESET;
  if ((BMI088_Write(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, BMI088_ACCEL_SOFTRESET_REG, 0xB6U) != HAL_OK) ||
      (BMI088_Write(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, BMI088_GYRO_SOFTRESET_REG, 0xB6U) != HAL_OK))
  {
    g_bmi088.error_count++;
    g_bmi088.init_status = (uint8_t)HAL_ERROR;
    return HAL_ERROR;
  }
  BMI088_DelayMs(50U);

  /* The first accel SPI read after a soft reset only restarts its SPI state. */
  g_bmi088.init_stage = BMI088_INIT_STAGE_RESTART_SPI;
  (void)BMI088_AccelRead(BMI088_ACCEL_CHIP_ID_REG, &discard, 1U);
  (void)BMI088_GyroRead(BMI088_GYRO_CHIP_ID_REG, &discard, 1U);

  g_bmi088.init_stage = BMI088_INIT_STAGE_CHECK_CHIP_IDS_AFTER_RESET;
  if (BMI088_CheckChipIds() != HAL_OK)
  {
    g_bmi088.error_count++;
    g_bmi088.init_status = (uint8_t)HAL_ERROR;
    return HAL_ERROR;
  }

  /* Accel and gyro data-ready signals are routed to PE10 and PE12 respectively. */
  g_bmi088.init_stage = BMI088_INIT_STAGE_CONFIGURE;
  if (BMI088_Write(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, BMI088_ACCEL_PWR_CTRL_REG, 0x04U) != HAL_OK)
  {
    g_bmi088.error_count++;
    g_bmi088.init_status = (uint8_t)HAL_ERROR;
    return HAL_ERROR;
  }
  BMI088_DelayMs(1U);

  if (BMI088_Write(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, BMI088_ACCEL_PWR_CONF_REG, 0x00U) != HAL_OK)
  {
    g_bmi088.error_count++;
    g_bmi088.init_status = (uint8_t)HAL_ERROR;
    return HAL_ERROR;
  }
  BMI088_DelayMs(1U);

  if ((BMI088_Write(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, BMI088_ACCEL_CONF_REG, 0xABU) != HAL_OK) ||
      (BMI088_Write(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, BMI088_ACCEL_RANGE_REG, 0x01U) != HAL_OK) ||
      (BMI088_Write(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, BMI088_ACCEL_INT1_IO_CTRL_REG, 0x08U) != HAL_OK) ||
      (BMI088_Write(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, BMI088_ACCEL_INT_MAP_DATA_REG, 0x04U) != HAL_OK) ||
      (BMI088_Write(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, BMI088_GYRO_RANGE_REG, 0x01U) != HAL_OK) ||
      (BMI088_Write(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, BMI088_GYRO_BANDWIDTH_REG, 0x02U) != HAL_OK) ||
      (BMI088_Write(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, BMI088_GYRO_INT_CTRL_REG, 0x80U) != HAL_OK) ||
      (BMI088_Write(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, BMI088_GYRO_INT3_INT4_IO_CONF_REG, 0x0CU) != HAL_OK) ||
      (BMI088_Write(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, BMI088_GYRO_INT3_INT4_IO_MAP_REG, 0x01U) != HAL_OK))
  {
    g_bmi088.error_count++;
    g_bmi088.init_status = (uint8_t)HAL_ERROR;
    return HAL_ERROR;
  }

  BMI088_DelayMs(10U);
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  bmi088_cpu_hz = HAL_RCC_GetSysClockFreq();
  bmi088_last_cycle = DWT->CYCCNT;
  g_bmi088.initialized = 1U;
  g_bmi088.init_stage = BMI088_INIT_STAGE_INITIAL_READ;
  status = BMI088_Read();
  if (status != HAL_OK)
  {
    g_bmi088.initialized = 0U;
    g_bmi088.init_status = (uint8_t)status;
    return status;
  }

  g_bmi088.init_stage = BMI088_INIT_STAGE_READY;
  g_bmi088.init_status = (uint8_t)HAL_OK;
  return HAL_OK;
}

HAL_StatusTypeDef BMI088_Read(void)
{
  uint8_t accel_data[6];
  uint8_t gyro_data[6];
  int16_t accel_raw[3];
  int16_t gyro_raw[3];

  if ((bmi088_hspi == NULL) || (g_bmi088.initialized == 0U) ||
      (BMI088_AccelRead(BMI088_ACCEL_DATA_REG, accel_data, sizeof(accel_data)) != HAL_OK) ||
      (BMI088_GyroRead(BMI088_GYRO_DATA_REG, gyro_data, sizeof(gyro_data)) != HAL_OK))
  {
    g_bmi088.error_count++;
    return HAL_ERROR;
  }

  for (uint32_t i = 0U; i < 3U; ++i)
  {
    accel_raw[i] = BMI088_ToInt16(&accel_data[i * 2U]);
    gyro_raw[i] = BMI088_ToInt16(&gyro_data[i * 2U]);
    g_bmi088.accel_mps2[i] = (float)accel_raw[i] * (6.0f * BMI088_GRAVITY_MPS2 / 32768.0f);
    g_bmi088.gyro_dps[i] = (float)gyro_raw[i] * (1000.0f / 32768.0f);
  }

  BMI088_UpdateAttitude();
  g_bmi088.update_count++;
  return HAL_OK;
}
