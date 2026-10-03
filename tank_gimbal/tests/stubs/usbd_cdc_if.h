#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
uint32_t HAL_GetTick(void);
uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len);
#ifdef __cplusplus
}
#endif
