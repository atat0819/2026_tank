#pragma once
#include <stdint.h>
struct USBD_CDC_HandleTypeDef { uint32_t TxState; };
struct USBD_HandleTypeDef { void* pClassData; };
extern "C" USBD_HandleTypeDef hUsbDeviceHS;
