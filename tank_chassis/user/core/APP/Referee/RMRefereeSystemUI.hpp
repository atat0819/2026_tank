#ifndef RM_REFEREE_SYSTEM_UI_HPP
#define RM_REFEREE_SYSTEM_UI_HPP

#include "RM_RefereeSystem.h"
#include <stdint.h>

namespace RMRefereeSystemUI
{
using Figure = RM_RefereeSystem::graphic_data_struct_t;
using Text = RM_RefereeSystem::ext_client_custom_character_t;

enum Operate : uint8_t
{
    OperateNull = 0,
    OperateAdd = 1,
    OperateRevise = 2,
    OperateDelete = 3,
};

enum Type : uint8_t
{
    TypeLine = RM_RefereeSystem::TypeLine,
    TypeRectangle = RM_RefereeSystem::TypeRectangle,
    TypeCircle = RM_RefereeSystem::TypeCircle,
    TypeElliptic = RM_RefereeSystem::TypeElliptic,
    TypeArced = RM_RefereeSystem::TypeArced,
    TypeFloat = RM_RefereeSystem::TypeFloat,
    TypeInt = RM_RefereeSystem::TypeInt,
    TypeString = RM_RefereeSystem::TypeStr,
};

enum Color : uint8_t
{
    ColorRedBlue = RM_RefereeSystem::ColorRedAndBlue,
    ColorYellow = RM_RefereeSystem::ColorYellow,
    ColorGreen = RM_RefereeSystem::ColorGreen,
    ColorOrange = RM_RefereeSystem::ColorOrange,
    ColorPurple = RM_RefereeSystem::ColorAmaranth,
    ColorPink = RM_RefereeSystem::ColorPink,
    ColorCyan = RM_RefereeSystem::ColorCyan,
    ColorBlack = RM_RefereeSystem::ColorBlack,
    ColorWhite = RM_RefereeSystem::ColorWhite,
};

struct Style
{
    uint16_t width;
    uint8_t color;
    uint8_t string_size;
    uint8_t operate_type;
};

void ResetStyle();
void SetColor(uint8_t color);
void SetWidth(uint16_t width);
void SetStringSize(uint8_t size);
void SetOperateType(uint8_t operate_type);
Style GetStyle();

Figure MakeLine(const char name[3], uint8_t layer, uint16_t start_x,
                uint16_t start_y, uint16_t end_x, uint16_t end_y);
Figure MakeRectangle(const char name[3], uint8_t layer, uint16_t start_x,
                     uint16_t start_y, uint16_t end_x, uint16_t end_y);
Figure MakeCircle(const char name[3], uint8_t layer, uint16_t center_x,
                  uint16_t center_y, uint16_t radius);
Figure MakeElliptic(const char name[3], uint8_t layer, uint16_t start_x,
                    uint16_t start_y, uint16_t end_x, uint16_t end_y);
Figure MakeArced(const char name[3], uint8_t layer, uint16_t start_angle,
                 uint16_t end_angle, uint16_t start_x, uint16_t start_y,
                 uint16_t end_x, uint16_t end_y);
Figure MakeFloat(const char name[3], uint8_t layer, float value,
                 uint16_t start_x, uint16_t start_y);
Figure MakeInt(const char name[3], uint8_t layer, int32_t value,
               uint16_t start_x, uint16_t start_y);
Text MakeString(const char name[3], uint8_t layer, const char *text,
                uint16_t start_x, uint16_t start_y);

void UpdatePowerStatus(float current_power, uint16_t max_power,
                       uint16_t buffer_energy);
void ResetPowerStatus();

} // namespace RMRefereeSystemUI

#endif // RM_REFEREE_SYSTEM_UI_HPP
