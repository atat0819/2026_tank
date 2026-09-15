#include "RMRefereeSystemUI.hpp"

#include <string.h>

namespace
{
RMRefereeSystemUI::Style g_style = {1, RMRefereeSystemUI::ColorRedBlue, 1,
                                    RMRefereeSystemUI::OperateAdd};
bool g_power_initialized = false;

void SetCommon(RMRefereeSystemUI::Figure &figure, const char name[3],
               uint8_t layer, uint8_t type)
{
    memset(&figure, 0, sizeof(figure));
    if (name != nullptr)
    {
        memcpy(figure.graphic_name, name, 3);
    }
    figure.operate_tpye = g_style.operate_type;
    figure.graphic_tpye = type;
    figure.layer = layer;
    figure.color = g_style.color;
    figure.width = g_style.width;
}

void SetSignedValue(RMRefereeSystemUI::Figure &figure, int32_t value)
{
    const uint32_t raw = static_cast<uint32_t>(value);
    figure.radius = raw & 0x3FFu;
    figure.end_x = (raw >> 10) & 0x7FFu;
    figure.end_y = (raw >> 21) & 0x7FFu;
}
} // namespace

namespace RMRefereeSystemUI
{
void ResetStyle()
{
    g_style = {1, ColorRedBlue, 1, OperateAdd};
}

void SetColor(uint8_t color) { g_style.color = color; }
void SetWidth(uint16_t width) { g_style.width = width; }
void SetStringSize(uint8_t size) { g_style.string_size = size; }
void SetOperateType(uint8_t operate_type) { g_style.operate_type = operate_type; }
Style GetStyle() { return g_style; }

Figure MakeLine(const char name[3], uint8_t layer, uint16_t start_x,
                uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    Figure figure;
    SetCommon(figure, name, layer, TypeLine);
    figure.start_x = start_x; figure.start_y = start_y;
    figure.end_x = end_x; figure.end_y = end_y;
    return figure;
}

Figure MakeRectangle(const char name[3], uint8_t layer, uint16_t start_x,
                     uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    Figure figure = MakeLine(name, layer, start_x, start_y, end_x, end_y);
    figure.graphic_tpye = TypeRectangle;
    return figure;
}

Figure MakeCircle(const char name[3], uint8_t layer, uint16_t center_x,
                  uint16_t center_y, uint16_t radius)
{
    Figure figure;
    SetCommon(figure, name, layer, TypeCircle);
    figure.start_x = center_x; figure.start_y = center_y;
    figure.radius = radius;
    return figure;
}

Figure MakeElliptic(const char name[3], uint8_t layer, uint16_t start_x,
                    uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    Figure figure = MakeLine(name, layer, start_x, start_y, end_x, end_y);
    figure.graphic_tpye = TypeElliptic;
    return figure;
}

Figure MakeArced(const char name[3], uint8_t layer, uint16_t start_angle,
                 uint16_t end_angle, uint16_t start_x, uint16_t start_y,
                 uint16_t end_x, uint16_t end_y)
{
    Figure figure = MakeLine(name, layer, start_x, start_y, end_x, end_y);
    figure.graphic_tpye = TypeArced;
    figure.start_angle = start_angle;
    figure.end_angle = end_angle;
    return figure;
}

Figure MakeFloat(const char name[3], uint8_t layer, float value,
                 uint16_t start_x, uint16_t start_y)
{
    Figure figure;
    SetCommon(figure, name, layer, TypeFloat);
    figure.start_angle = g_style.string_size;
    figure.end_angle = 3;
    figure.start_x = start_x; figure.start_y = start_y;
    SetSignedValue(figure, static_cast<int32_t>(value * 1000.0f));
    return figure;
}

Figure MakeInt(const char name[3], uint8_t layer, int32_t value,
               uint16_t start_x, uint16_t start_y)
{
    Figure figure;
    SetCommon(figure, name, layer, TypeInt);
    figure.start_angle = g_style.string_size;
    figure.start_x = start_x; figure.start_y = start_y;
    SetSignedValue(figure, value);
    return figure;
}

Text MakeString(const char name[3], uint8_t layer, const char *text,
                uint16_t start_x, uint16_t start_y)
{
    Text result = {};
    SetCommon(result.grapic_data_struct, name, layer, TypeString);
    result.grapic_data_struct.start_angle = g_style.string_size;
    result.grapic_data_struct.start_x = start_x;
    result.grapic_data_struct.start_y = start_y;
    if (text != nullptr)
    {
        size_t length = strlen(text);
        if (length > sizeof(result.data)) length = sizeof(result.data);
        result.grapic_data_struct.end_angle = length;
        memcpy(result.data, text, length);
    }
    return result;
}

void UpdatePowerStatus(float current_power, uint16_t max_power,
                       uint16_t buffer_energy)
{
    SetOperateType(g_power_initialized ? OperateRevise : OperateAdd);
    SetColor(ColorCyan);
    SetWidth(2);
    SetStringSize(2);

    if (!g_power_initialized)
    {
        RM_RefereeSystem::RM_RefereeSystemSendStr(
            MakeString("PW_", 1, "PWR", 1320, 160));
        RM_RefereeSystem::RM_RefereeSystemSendStr(
            MakeString("MX_", 1, "MAX", 1320, 220));
        RM_RefereeSystem::RM_RefereeSystemSendStr(
            MakeString("BF_", 1, "BUF", 1320, 280));
    }

    Figure power[2];
    const int32_t rounded_power = static_cast<int32_t>(
        current_power + (current_power >= 0.0f ? 0.5f : -0.5f));
    power[0] = MakeInt("PWR", 1, rounded_power, 1450, 160);
    power[1] = MakeInt("MAX", 1, static_cast<int32_t>(max_power), 1450, 220);
    RM_RefereeSystem::RM_RefereeSystemSendDataN(power, 2);

    Figure buffer = MakeInt("BUF", 1, static_cast<int32_t>(buffer_energy),
                            1450, 280);
    RM_RefereeSystem::RM_RefereeSystemSendDataN(&buffer, 1);

    g_power_initialized = true;
}

void ResetPowerStatus()
{
    g_power_initialized = false;
}
} // namespace RMRefereeSystemUI
