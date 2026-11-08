#include "RMRefereeSystemUI.hpp"

#include "cmsis_os.h"
#include <string.h>

namespace
{
RMRefereeSystemUI::Style g_style = {1, RMRefereeSystemUI::ColorRedBlue, 1,
                                    RMRefereeSystemUI::OperateAdd};
bool g_power_initialized = false;
bool g_health_initialized = false;
bool g_health_bar_visible = false;
bool g_aim_initialized = false;
bool g_ui_sent = false;
uint32_t g_last_send_tick = 0;
constexpr uint32_t UI_TX_INTERVAL_MS = 40;

void WaitForUiSlot()
{
    if (!g_ui_sent) return;
    const uint32_t elapsed = HAL_GetTick() - g_last_send_tick;
    if (elapsed < UI_TX_INTERVAL_MS) osDelay(UI_TX_INTERVAL_MS - elapsed);
}

bool RecordUiSend(bool sent)
{
    if (sent)
    {
        g_ui_sent = true;
        g_last_send_tick = HAL_GetTick();
    }
    return sent;
}

bool SendFigures(const RMRefereeSystemUI::Figure *figures, int count)
{
    WaitForUiSlot();
    return RecordUiSend(RM_RefereeSystem::RM_RefereeSystemSendDataN(figures, count));
}

bool SendText(const RMRefereeSystemUI::Text &text)
{
    WaitForUiSlot();
    return RecordUiSend(RM_RefereeSystem::RM_RefereeSystemSendStr(text));
}

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
        if (!SendText(
                MakeString("PW_", 1, "PWR", 1320, 160))) return;
        if (!SendText(
                MakeString("MX_", 1, "MAX", 1320, 220))) return;
        if (!SendText(
                MakeString("BF_", 1, "BUF", 1320, 280))) return;
    }

    Figure power[2];
    const int32_t rounded_power = static_cast<int32_t>(
        current_power + (current_power >= 0.0f ? 0.5f : -0.5f));
    power[0] = MakeInt("PWR", 1, rounded_power, 1450, 160);
    power[1] = MakeInt("MAX", 1, static_cast<int32_t>(max_power), 1450, 220);
    if (!SendFigures(power, 2)) return;

    Figure buffer = MakeInt("BUF", 1, static_cast<int32_t>(buffer_energy),
                            1450, 280);
    if (!SendFigures(&buffer, 1)) return;

    g_power_initialized = true;
}

void ResetPowerStatus()
{
    g_power_initialized = false;
}

void UpdateHealthStatus(uint16_t current_hp, uint16_t maximum_hp)
{
    SetOperateType(g_health_initialized ? OperateRevise : OperateAdd);
    SetColor(ColorGreen);
    SetWidth(2);
    SetStringSize(24);

    if (!g_health_initialized)
    {
        if (!SendText(MakeString("HP_", 2, "HP", 100, 870))) return;
        Figure border = MakeRectangle("HPB", 2, 100, 800, 400, 824);
        if (!SendFigures(&border, 1)) return;
    }

    Figure health[2];
    health[0] = MakeInt("HPN", 2, current_hp, 180, 870);

    // Round up so a positive HP remains visible; never exceed the border.
    const uint16_t bounded_hp = current_hp > maximum_hp ? maximum_hp : current_hp;
    const uint16_t fill_length = maximum_hp == 0 ? 0 :
        static_cast<uint16_t>((292U * bounded_hp + maximum_hp - 1U) / maximum_hp);
    SetOperateType(fill_length == 0 ? OperateDelete :
                   g_health_bar_visible ? OperateRevise : OperateAdd);
    SetWidth(16);
    health[1] = MakeLine("HPF", 2, 104, 812, 104 + fill_length, 812);
    if (!SendFigures(health, 2)) return;

    g_health_initialized = true;
    g_health_bar_visible = fill_length != 0;
}

void ResetHealthStatus()
{
    g_health_initialized = false;
    g_health_bar_visible = false;
}

void UpdateAimGuides()
{
    if (g_aim_initialized) return;
    SetOperateType(OperateAdd);
    SetColor(ColorGreen);
    SetWidth(2);
    SetStringSize(20);

    using namespace AimConfig;
    Figure guides[5];
    guides[0] = MakeLine("CRH", 3, CENTER_X - CROSS_HALF_WIDTH, CENTER_Y,
                         CENTER_X + CROSS_HALF_WIDTH, CENTER_Y);
    guides[1] = MakeLine("CRV", 3, CENTER_X, CENTER_Y - CROSS_HALF_HEIGHT,
                         CENTER_X, CENTER_Y + CROSS_HALF_HEIGHT);
    for (uint8_t i = 0; i < 3; ++i)
    {
        const DistanceMark &mark = DISTANCE_MARKS[i];
        const int16_t x = CENTER_X + mark.x_offset;
        const int16_t y = CENTER_Y + mark.y_offset;
        const char name[3] = {'D', 'T', static_cast<char>('0' + i)};
        guides[2 + i] = MakeLine(name, 3, x - mark.half_width, y,
                                  x + mark.half_width, y);
    }
    if (!SendFigures(guides, 5)) return;

    for (uint8_t i = 0; i < 3; ++i)
    {
        const DistanceMark &mark = DISTANCE_MARKS[i];
        const char name[3] = {'D', 'L', static_cast<char>('0' + i)};
        if (!SendText(MakeString(name, 3, mark.label,
                CENTER_X + mark.x_offset + mark.half_width + LABEL_OFFSET_X,
                CENTER_Y + mark.y_offset + LABEL_OFFSET_Y))) return;
    }
    g_aim_initialized = true;
}

void ResetAimGuides()
{
    g_aim_initialized = false;
}
} // namespace RMRefereeSystemUI
