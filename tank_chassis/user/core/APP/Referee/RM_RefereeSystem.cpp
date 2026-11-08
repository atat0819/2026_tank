#include "RM_RefereeSystem.h"
#include "cmsis_os.h"
#include "memory"
#include "string.h"
using namespace RM_RefereeSystem;
using namespace RM_RefereeSystemCRC;
namespace
{
constexpr uint32_t REFEREE_TIMEOUT_MS = 1000;
volatile uint32_t referee_status_tick = 0;
volatile uint32_t referee_heat_tick = 0;
volatile bool referee_status_received = false;
volatile bool referee_heat_received = false;

bool WaitForRefereeTx()
{
    const uint32_t start = HAL_GetTick();
    auto *handle = HAL::UART::get_uart_bus_instance().get_uart1().get_handle();
    // RX DMA常驻BUSY_RX；这里只等待TX，最长20ms（127字节约需11ms）。
    while (handle->gState != HAL_UART_STATE_READY)
    {
        if (static_cast<uint32_t>(HAL_GetTick() - start) >= 20U)
            return false;
        osDelay(1);
    }
    return true;
}
}
// 启动时没有有效的性能/热量数据。
bool RM_RefereeSystemDirFlag = true;
// 接收数据
uint8_t RM_RefereeSystemp8Data = 0;
// 设置颜色与线粗
RM_RefereeSystemToop_t RM_RefereeSystemToop = {1, 1, 1, 1};
// 数据接收格式变量
RM_RefereeSystemData_t RM_RefereeSystemData = {0};

RM_RefereeSystemData_t RM_RefereeSystemData01 = {0};
// 0x0201 机器人状态数据
ext_game_robot_status_t ext_power_heat_data_0x0201 = {0};
// 0x0202 实时功率热量数据
ext_power_heat_data_t ext_power_heat_data_0x0202 = {0};
volatile uint8_t referee_0202_raw_heat[2] = {0, 0};
volatile uint16_t referee_0202_heat_after_memcpy = 0;
volatile uint16_t referee_0202_data_length = 0;
// 0x0303 机器人间交互数据
map_command_t map_command_0x0303 = {0};

// 发送数据
uint8_t tx_buf[128] = {0};
uint8_t rrrdata[57] = {0};

uint8_t daohang_tx_buf[128] = {0};

// 引用 remote_task.cpp 全局作用域定义的变量
extern HAL::UART::Data uart1_rx_data;
extern uint8_t referee_buffer[512];

namespace RM_RefereeSystem
{
// 初始化（DMA+空闲模式由 remote_task 管理，此处保留用于断连重连）
void RM_RefereeSystemInit()
{
    uart1_rx_data.buffer = referee_buffer;
    uart1_rx_data.size   = sizeof(referee_buffer);
    HAL::UART::get_uart_bus_instance().get_uart1().receive_dma_idle(uart1_rx_data);
}
// 设置颜色
void RM_RefereeSystemSetColor(int color)
{
    RM_RefereeSystemToop.color = color;
}
// 设置线粗
void RM_RefereeSystemSetWidth(int width)
{
    RM_RefereeSystemToop.width = width;
}
// 设置字体
void RM_RefereeSystemSetStringSize(int size)
{
    RM_RefereeSystemToop.strsize = size;
}
// 设置图形操作
void RM_RefereeSystemSetOperateTpye(int operate_tpye)
{
    RM_RefereeSystemToop.operate_tpye = operate_tpye;
}
// 工具清空
void RM_RefereeSystemClsToop()
{
    RM_RefereeSystemToop.color = RM_RefereeSystemToop.width = RM_RefereeSystemToop.strsize =
        RM_RefereeSystemToop.operate_tpye = 1;
}
// 机器人获取id
int RM_RefereeSystemGetRobotId()
{
    uint16_t receiver_ID = 0x0000;
    // 红色
    if (ext_power_heat_data_0x0201.robot_id < 100)
    {
        switch (ext_power_heat_data_0x0201.robot_id)
        {
        case 1: /*英雄机器人*/
            receiver_ID = 0x0101;
            break;
        case 2: /*工程机器人*/
            receiver_ID = 0x0102;
            break;
        case 3: /*步兵机器人*/
            receiver_ID = 0x0103;
            break;
        case 4: /*步兵机器人*/
            receiver_ID = 0x0104;
            break;
        case 5: /*步兵机器人*/
            receiver_ID = 0x0105;
            break;
        case 6: /*空中机器人*/
            receiver_ID = 0x0106;
            break;
        }
    }
    // 蓝色
    if (ext_power_heat_data_0x0201.robot_id > 100)
    {
        switch (ext_power_heat_data_0x0201.robot_id)
        {
        case 101: /*英雄机器人*/
            receiver_ID = 0x0165;
            break;
        case 102: /*工程机器人*/
            receiver_ID = 0x0166;
            break;
        case 103: /*步兵机器人*/
            receiver_ID = 0x0167;
            break;
        case 104: /*步兵机器人*/
            receiver_ID = 0x0168;
            break;
        case 105: /*步兵机器人*/
            receiver_ID = 0x0169;
            break;
        case 106: /*空中机器人*/
            receiver_ID = 0x016A;
            break;
        }
    }
    return receiver_ID;
}
// 绘画直线/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetLine(char *name, uint32_t layer, uint32_t start_x, uint32_t start_y,
                                              uint32_t end_x, uint32_t end_y)
{
    graphic_data_struct_t graphic_data_struct = {0};
    memcpy((void *)graphic_data_struct.graphic_name, name, 3);
    graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
    graphic_data_struct.graphic_tpye = TypeLine;
    graphic_data_struct.layer = layer;
    graphic_data_struct.color = RM_RefereeSystemToop.color;
    graphic_data_struct.width = RM_RefereeSystemToop.width;
    graphic_data_struct.start_x = start_x;
    graphic_data_struct.start_y = start_y;
    graphic_data_struct.end_x = end_x;
    graphic_data_struct.end_y = end_y;
    return graphic_data_struct;
}
// 绘画矩形/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetRectangle(char *name, uint32_t layer, uint32_t start_x, uint32_t start_y,
                                                   uint32_t end_x, uint32_t end_y)
{
    graphic_data_struct_t graphic_data_struct = {0};
    memcpy((void *)graphic_data_struct.graphic_name, name, 3);
    graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
    graphic_data_struct.graphic_tpye = TypeRectangle;
    graphic_data_struct.layer = layer;
    graphic_data_struct.color = RM_RefereeSystemToop.color;
    graphic_data_struct.width = RM_RefereeSystemToop.width;
    graphic_data_struct.start_x = start_x;
    graphic_data_struct.start_y = start_y;
    graphic_data_struct.end_x = end_x;
    graphic_data_struct.end_y = end_y;
    return graphic_data_struct;
}
// 绘画圆/*名字，图层，x1，y1，半径*/
graphic_data_struct_t RM_RefereeSystemSetCircle(char *name, uint32_t layer, uint32_t start_x, uint32_t start_y,
                                                uint32_t radius)
{
    graphic_data_struct_t graphic_data_struct = {0};
    memcpy((void *)graphic_data_struct.graphic_name, name, 3);
    graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
    graphic_data_struct.graphic_tpye = TypeCircle;
    graphic_data_struct.layer = layer;
    graphic_data_struct.color = RM_RefereeSystemToop.color;
    graphic_data_struct.width = RM_RefereeSystemToop.width;
    graphic_data_struct.start_x = start_x;
    graphic_data_struct.start_y = start_y;
    graphic_data_struct.radius = radius;
    return graphic_data_struct;
}
// 绘画椭圆/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetElliptic(char *name, uint32_t layer, uint32_t start_x, uint32_t start_y,
                                                  uint32_t end_x, uint32_t end_y)
{
    graphic_data_struct_t graphic_data_struct = {0};
    memcpy((void *)graphic_data_struct.graphic_name, name, 3);
    graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
    graphic_data_struct.graphic_tpye = TypeElliptic;
    graphic_data_struct.layer = layer;
    graphic_data_struct.color = RM_RefereeSystemToop.color;
    graphic_data_struct.width = RM_RefereeSystemToop.width;
    graphic_data_struct.start_x = start_x;
    graphic_data_struct.start_y = start_y;
    graphic_data_struct.end_x = end_x;
    graphic_data_struct.end_y = end_y;
    return graphic_data_struct;
}
// 绘画圆弧/*名字，图层，起始角度，终止角度，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetArced(char *name, uint32_t layer, uint32_t start_angle, uint32_t end_angle,
                                               uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
    graphic_data_struct_t graphic_data_struct = {0};
    memcpy((void *)graphic_data_struct.graphic_name, name, 3);
    graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
    graphic_data_struct.graphic_tpye = TypeArced;
    graphic_data_struct.layer = layer;
    graphic_data_struct.color = RM_RefereeSystemToop.color;
    graphic_data_struct.width = RM_RefereeSystemToop.width;
    graphic_data_struct.start_angle = start_angle;
    graphic_data_struct.end_angle = end_angle;
    graphic_data_struct.start_x = start_x;
    graphic_data_struct.start_y = start_y;
    graphic_data_struct.end_x = end_x;
    graphic_data_struct.end_y = end_y;
    return graphic_data_struct;
}
// 绘画浮点数/*名字，图层，浮点数，浮点数长度，x1，y1*/
graphic_data_struct_t RM_RefereeSystemSetFzloat(char *name, uint32_t layer, float x, uint32_t start_x, uint32_t start_y)
{
    graphic_data_struct_t graphic_data_struct = {0};
    memcpy((void *)graphic_data_struct.graphic_name, name, 3);
    graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
    graphic_data_struct.graphic_tpye = TypeFloat;
    graphic_data_struct.layer = layer;
    graphic_data_struct.color = RM_RefereeSystemToop.color;
    graphic_data_struct.width = RM_RefereeSystemToop.width;
    graphic_data_struct.start_angle = RM_RefereeSystemToop.strsize;
    graphic_data_struct.end_angle = 3;
    graphic_data_struct.start_x = start_x;
    graphic_data_struct.start_y = start_y;
    graphic_data_struct.radius |= ((int32_t)(x * 1000) >> 0) & 0x3ff;
    graphic_data_struct.end_x |= ((int32_t)(x * 1000) >> 10) & 0x7ff;
    graphic_data_struct.end_y |= ((int32_t)(x * 1000) >> (10 + 11)) & 0x7ff;
    return graphic_data_struct;
}
// 绘画整数/*名字，图层，整数，浮点数长度，x1，y1*/
graphic_data_struct_t RM_RefereeSystemSetInt(char *name, uint32_t layer, int32_t x, uint32_t start_x, uint32_t start_y)
{
    graphic_data_struct_t graphic_data_struct = {0};
    memcpy((void *)graphic_data_struct.graphic_name, name, 3);
    graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
    graphic_data_struct.graphic_tpye = TypeInt;
    graphic_data_struct.layer = layer;
    graphic_data_struct.color = RM_RefereeSystemToop.color;
    graphic_data_struct.width = RM_RefereeSystemToop.width;
    graphic_data_struct.start_angle = RM_RefereeSystemToop.strsize;
    graphic_data_struct.start_x = start_x;
    graphic_data_struct.start_y = start_y;
    graphic_data_struct.radius |= (x >> 0) & 0x3ff;
    graphic_data_struct.end_x |= (x >> 10) & 0x7ff;
    graphic_data_struct.end_y |= (x >> (10 + 11)) & 0x7ff;
    return graphic_data_struct;
}
// 绘画字符串/*名字，图层，字符串，x1，y1*/
ext_client_custom_character_t RM_RefereeSystemSetStr(char *name, uint32_t layer, char *str, uint32_t start_x,
                                                     uint32_t start_y)
{
    ext_client_custom_character_t ext_client_custom_character = {0};
    memcpy((void *)ext_client_custom_character.grapic_data_struct.graphic_name, name, 3);
    ext_client_custom_character.grapic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
    ext_client_custom_character.grapic_data_struct.graphic_tpye = TypeStr;
    ext_client_custom_character.grapic_data_struct.layer = layer;
    ext_client_custom_character.grapic_data_struct.color = RM_RefereeSystemToop.color;
    ext_client_custom_character.grapic_data_struct.width = RM_RefereeSystemToop.width;
    ext_client_custom_character.grapic_data_struct.start_angle = RM_RefereeSystemToop.strsize;
    size_t length = strlen(str);
    if (length > sizeof(ext_client_custom_character.data))
        length = sizeof(ext_client_custom_character.data);
    ext_client_custom_character.grapic_data_struct.end_angle = length;
    ext_client_custom_character.grapic_data_struct.start_x = start_x;
    ext_client_custom_character.grapic_data_struct.start_y = start_y;
    memcpy((void *)ext_client_custom_character.data, str, ext_client_custom_character.grapic_data_struct.end_angle);
    return ext_client_custom_character;
}
// 数据发送客户端绘制删除N个图层
bool RM_RefereeSystemDelete(const char operate, const char number)
{
    if (!WaitForRefereeTx()) return false;

    static uint8_t seq = 0;
    RM_RefereeSystemData_t RM_RefereeSystemDataTemp = {0};
    // id设置
    ext_student_interactive_header_data_t ext_student_interactive_header_data = {0};
    ext_student_interactive_header_data.data_cmd_id = 0x0100;
    ext_student_interactive_header_data.sender_ID = ext_power_heat_data_0x0201.robot_id;
    ext_student_interactive_header_data.receiver_ID = RM_RefereeSystemGetRobotId();
    // 数据格式
    RM_RefereeSystemDataTemp.SOF = 0xA5;
    RM_RefereeSystemDataTemp.data_length = 6 + 2;
    RM_RefereeSystemDataTemp.seq = seq++;
    RM_RefereeSystemDataTemp.cmd_id = 0x0301;
    memset(&tx_buf, 0x00, sizeof(tx_buf));
    // 数据=id数据+删除图像
    int idx = 0;
    memcpy((void *)RM_RefereeSystemDataTemp.data, &ext_student_interactive_header_data,
           sizeof(ext_student_interactive_header_data));
    idx += sizeof(ext_student_interactive_header_data);
    memcpy((void *)(RM_RefereeSystemDataTemp.data + idx), &operate, sizeof(operate));
    idx += sizeof(operate);
    memcpy((void *)(RM_RefereeSystemDataTemp.data + idx), &number, sizeof(number));
    memcpy(tx_buf, &RM_RefereeSystemDataTemp, sizeof(RM_RefereeSystemDataTemp));
    Append_CRC8_Check_Sum(tx_buf, CRC8LEN);
    Append_CRC16_Check_Sum(tx_buf, CRC16LEN(RM_RefereeSystemDataTemp.data_length));
    HAL::UART::Data tx_data{tx_buf,
                            static_cast<uint16_t>(RM_RefereeSystemDataTemp.data_length + 9)};
    return HAL::UART::get_uart_bus_instance().get_uart1().transmit_dma(tx_data);
}
// 数据发送客户端绘制一个图形
bool RM_RefereeSystemSendData1(const graphic_data_struct_t graphic_data_struct)
{
    if (!WaitForRefereeTx()) return false;

    static uint8_t seq = 0;
    RM_RefereeSystemData_t RM_RefereeSystemDataTemp = {0};
    // id设置
    ext_student_interactive_header_data_t ext_student_interactive_header_data = {0};
    ext_student_interactive_header_data.data_cmd_id = 0x0101;
    ext_student_interactive_header_data.sender_ID = ext_power_heat_data_0x0201.robot_id;
    ext_student_interactive_header_data.receiver_ID = RM_RefereeSystemGetRobotId();
    // 数据格式
    RM_RefereeSystemDataTemp.SOF = 0xA5;
    RM_RefereeSystemDataTemp.data_length = 6 + 15;
    RM_RefereeSystemDataTemp.seq = seq++;
    RM_RefereeSystemDataTemp.cmd_id = 0x0301;
    memset(&tx_buf, 0x00, sizeof(tx_buf));
    // 数据=id数据+图像数据
    int idx = 0;
    memcpy((void *)RM_RefereeSystemDataTemp.data, &ext_student_interactive_header_data,
           sizeof(ext_student_interactive_header_data));
    idx += sizeof(ext_student_interactive_header_data);
    memcpy((void *)(RM_RefereeSystemDataTemp.data + idx), &graphic_data_struct, sizeof(graphic_data_struct));
    memcpy(tx_buf, &RM_RefereeSystemDataTemp, sizeof(RM_RefereeSystemDataTemp));
    Append_CRC8_Check_Sum(tx_buf, CRC8LEN);
    Append_CRC16_Check_Sum(tx_buf, CRC16LEN(RM_RefereeSystemDataTemp.data_length));
    HAL::UART::Data tx_data{tx_buf,
                            static_cast<uint16_t>(RM_RefereeSystemDataTemp.data_length + 9)};
    return HAL::UART::get_uart_bus_instance().get_uart1().transmit_dma(tx_data);
}
// 数据发送客户端绘制1,2,5,7个图形
bool RM_RefereeSystemSendDataN(const graphic_data_struct_t graphic_data_struct[], int size)
{
    if (graphic_data_struct == nullptr ||
        (size != 1 && size != 2 && size != 5 && size != 7)) return false;
    if (!WaitForRefereeTx()) return false;

    static uint8_t seq = 0;
    RM_RefereeSystemData_t RM_RefereeSystemDataTemp = {0};
    // id设置
    ext_student_interactive_header_data_t ext_student_interactive_header_data = {0};
    switch (size)
    {
    case 1:
        ext_student_interactive_header_data.data_cmd_id = 0x0101;
        break;
    case 2:
        ext_student_interactive_header_data.data_cmd_id = 0x0102;
        break;
    case 5:
        ext_student_interactive_header_data.data_cmd_id = 0x0103;
        break;
    case 7:
        ext_student_interactive_header_data.data_cmd_id = 0x0104;
        break;
    }
    ext_student_interactive_header_data.sender_ID = ext_power_heat_data_0x0201.robot_id;
    ext_student_interactive_header_data.receiver_ID = RM_RefereeSystemGetRobotId();
    // 数据格式
    RM_RefereeSystemDataTemp.SOF = 0xA5;
    switch (size)
    {
    case 1:
        RM_RefereeSystemDataTemp.data_length = 6 + 15;
        break;
    case 2:
        RM_RefereeSystemDataTemp.data_length = 6 + 30;
        break;
    case 5:
        RM_RefereeSystemDataTemp.data_length = 6 + 75;
        break;
    case 7:
        RM_RefereeSystemDataTemp.data_length = 6 + 105;
        break;
    }
    RM_RefereeSystemDataTemp.seq = seq++;
    RM_RefereeSystemDataTemp.cmd_id = 0x0301;
    memset(&tx_buf, 0, sizeof(tx_buf));
    // 数据=id数据+图像数据
    int idx = 0;
    memcpy((void *)RM_RefereeSystemDataTemp.data, &ext_student_interactive_header_data,
           sizeof(ext_student_interactive_header_data));
    idx += sizeof(ext_student_interactive_header_data);
    for (int i = 0; i < size; i++)
    {
        memcpy((void *)(RM_RefereeSystemDataTemp.data + idx), &graphic_data_struct[i], sizeof(graphic_data_struct_t));
        idx += sizeof(graphic_data_struct_t);
    }
    memcpy(tx_buf, &RM_RefereeSystemDataTemp, sizeof(RM_RefereeSystemDataTemp));
    Append_CRC8_Check_Sum(tx_buf, CRC8LEN);
    Append_CRC16_Check_Sum(tx_buf, CRC16LEN(RM_RefereeSystemDataTemp.data_length));
    HAL::UART::Data tx_data{tx_buf,
                            static_cast<uint16_t>(RM_RefereeSystemDataTemp.data_length + 9)};
    return HAL::UART::get_uart_bus_instance().get_uart1().transmit_dma(tx_data);
}
// 数据发送客户端绘制字符串
bool RM_RefereeSystemSendStr(const ext_client_custom_character_t ext_client_custom_character)
{
    if (ext_client_custom_character.grapic_data_struct.end_angle >
        sizeof(ext_client_custom_character.data)) return false;
    if (!WaitForRefereeTx()) return false;

    static uint8_t seq = 0;
    RM_RefereeSystemData_t RM_RefereeSystemDataTemp = {0};
    // id设置
    ext_student_interactive_header_data_t ext_student_interactive_header_data = {0};
    ext_student_interactive_header_data.data_cmd_id = 0x0110;
    ext_student_interactive_header_data.sender_ID = ext_power_heat_data_0x0201.robot_id;
    ext_student_interactive_header_data.receiver_ID = RM_RefereeSystemGetRobotId();
    // 数据格式
    RM_RefereeSystemDataTemp.SOF = 0xA5;
    RM_RefereeSystemDataTemp.data_length = 6 + 45;
    RM_RefereeSystemDataTemp.seq = seq++;
    RM_RefereeSystemDataTemp.cmd_id = 0x0301;
    memset(&tx_buf, 0, sizeof(tx_buf));
    // 数据=id数据+图像数据
    int idx = 0;
    memcpy((void *)RM_RefereeSystemDataTemp.data, &ext_student_interactive_header_data,
           sizeof(ext_student_interactive_header_data));
    idx += sizeof(ext_student_interactive_header_data);
    memcpy((void *)(RM_RefereeSystemDataTemp.data + idx), &ext_client_custom_character.grapic_data_struct,
           sizeof(ext_client_custom_character.grapic_data_struct));
    idx += sizeof(ext_client_custom_character.grapic_data_struct);
    memcpy((void *)(RM_RefereeSystemDataTemp.data + idx), (void *)ext_client_custom_character.data,
           ext_client_custom_character.grapic_data_struct.end_angle);
    memcpy(tx_buf, &RM_RefereeSystemDataTemp, sizeof(RM_RefereeSystemDataTemp));
    Append_CRC8_Check_Sum(tx_buf, CRC8LEN);
    Append_CRC16_Check_Sum(tx_buf, CRC16LEN(RM_RefereeSystemDataTemp.data_length));
    HAL::UART::Data tx_data{tx_buf,
                            static_cast<uint16_t>(RM_RefereeSystemDataTemp.data_length + 9)};
    return HAL::UART::get_uart_bus_instance().get_uart1().transmit_dma(tx_data);
}

/*typedef __packed struct
{
uint16_t target_robot_id;
float target_position_x;
float target_position_y;
}map_robot_data_t;
#define CRC8LEN (5)
#define CRC16LEN(x) (x + 5 + 2 + 2)*/
map_robot_data_t map_robot_data;
void RM_daohangtx(const map_robot_data_t map_robot_data)
{
    // 2027协议只允许雷达通过0x0305发送双方坐标。
    const uint8_t robot_id = ext_power_heat_data_0x0201.robot_id;
    if (robot_id != 9 && robot_id != 109) return;
    if (!WaitForRefereeTx()) return;

    static uint8_t seq = 0;
    RM_RefereeSystemData_t RM_RefereeSystemDataTemp = {0};
    // 数据格式
    RM_RefereeSystemDataTemp.SOF = 0xA5;
    RM_RefereeSystemDataTemp.data_length = sizeof(map_robot_data);
    RM_RefereeSystemDataTemp.seq = seq++;
    RM_RefereeSystemDataTemp.cmd_id = 0x0305;
    memset(&rrrdata, 0, sizeof(rrrdata));

    memcpy((void *)(RM_RefereeSystemDataTemp.data), &map_robot_data, sizeof(map_robot_data));
    memcpy(rrrdata, &RM_RefereeSystemDataTemp, sizeof(rrrdata));
    Append_CRC8_Check_Sum(rrrdata, CRC8LEN);
    Append_CRC16_Check_Sum(rrrdata, CRC16LEN(RM_RefereeSystemDataTemp.data_length));
    
    HAL::UART::Data nav_data{rrrdata,
                            static_cast<uint16_t>(RM_RefereeSystemDataTemp.data_length + 9)};
    HAL::UART::get_uart_bus_instance().get_uart1().transmit_dma(nav_data);
}

void RM_daohangtxParsea(uint8_t *data)
{
    RM_RefereeSystemData01.SOF = data[0];
    RM_RefereeSystemData01.data_length = data[1] | data[2] << 8;
    RM_RefereeSystemData01.seq = data[3];
    RM_RefereeSystemData01.CRC8 = data[4];
    RM_RefereeSystemData01.cmd_id = data[5] | data[6] << 8;
    for (uint16_t i = 0; i < RM_RefereeSystemData01.data_length; i++)
    {
        RM_RefereeSystemData01.data[i] = data[7 + i];
    }
    if (!Verify_CRC8_Check_Sum(data, CRC8LEN) ||
        !Verify_CRC16_Check_Sum(data, CRC16LEN(RM_RefereeSystemData01.data_length)))
        return;
    // memcpy(&map_robot_data_1,(void*)RM_RefereeSystemData01.data,sizeof(map_robot_data_1));
}

// 数据解析
void RM_RefereeSystemParseData(uint8_t *RM_pDatas, int size)
{
    if (RM_pDatas == nullptr || size < 9 || RM_pDatas[0] != 0xA5)
        return;

    const uint16_t data_length = RM_pDatas[1] | RM_pDatas[2] << 8;
    const uint16_t cmd_id = RM_pDatas[5] | RM_pDatas[6] << 8;
    if (data_length > sizeof(RM_RefereeSystemData.data) ||
        size != CRC16LEN(data_length))
        return;
    if (!Verify_CRC8_Check_Sum(RM_pDatas, CRC8LEN) ||
        !Verify_CRC16_Check_Sum(RM_pDatas, CRC16LEN(data_length)))
        return;

    // 固定长度命令只接受完整的2027数据段，禁止旧值拼接或短包清零。
    switch (cmd_id)
    {
    case 0x0201:
        if (data_length != sizeof(ext_power_heat_data_0x0201)) return;
        break;
    case 0x0202:
        if (data_length != sizeof(ext_power_heat_data_0x0202)) return;
        break;
    case 0x0303:
        if (data_length != sizeof(map_command_0x0303)) return;
        break;
    default:
        return;
    }
    RM_RefereeSystemData.SOF = RM_pDatas[0];
    RM_RefereeSystemData.data_length = data_length;
    RM_RefereeSystemData.seq = RM_pDatas[3];
    RM_RefereeSystemData.CRC8 = RM_pDatas[4];
    RM_RefereeSystemData.cmd_id = cmd_id;
    memcpy(RM_RefereeSystemData.data, RM_pDatas + 7, data_length);
    RM_RefereeSystemData.frame_tail = RM_pDatas[7 + data_length] |
                                      RM_pDatas[8 + data_length] << 8;
    switch (cmd_id)
    {
    case 0x0201:
        memcpy(&ext_power_heat_data_0x0201, (void *)RM_RefereeSystemData.data, sizeof(ext_power_heat_data_0x0201));
        referee_status_tick = HAL_GetTick();
        referee_status_received = true;
        break;
    case 0x0202:
    {
        referee_0202_data_length = RM_RefereeSystemData.data_length;
        // 2027 V1.0.0：42mm热量位于偏移12、13，总长度14字节。
        memcpy(&ext_power_heat_data_0x0202,
               (void *)RM_RefereeSystemData.data,
               sizeof(ext_power_heat_data_0x0202));
        referee_0202_raw_heat[0] = RM_RefereeSystemData.data[12];
        referee_0202_raw_heat[1] = RM_RefereeSystemData.data[13];
        referee_0202_heat_after_memcpy = ext_power_heat_data_0x0202.shooter_id1_42mm_cooling_heat;
        referee_heat_tick = HAL_GetTick();
        referee_heat_received = true;
        break;
    }
    case 0x0303:
        memcpy(&map_command_0x0303, (void *)RM_RefereeSystemData.data, sizeof(map_command_0x0303));
        break;
    default:
        break;
    }
}
// 消息获取（状态机模式）
void RM_RefereeSystemGetData(uint8_t byte)
{
    static uint8_t RxBuffer[256]; // 增加缓冲区大小
    static uint16_t RxIndex = 0;
    static uint16_t DataLen = 0;
    static enum {
        STATE_SOF,
        STATE_LEN_LOW,
        STATE_LEN_HIGH,
        STATE_SEQ,
        STATE_CRC8,
        STATE_CMD_ID_LOW,
        STATE_CMD_ID_HIGH,
        STATE_DATA,
        STATE_TAIL_LOW,
        STATE_TAIL_HIGH
    } State = STATE_SOF;

    switch (State)
    {
    case STATE_SOF:
        if (byte == 0xA5)
        {
            RxBuffer[0] = byte;
            RxIndex = 1;
            State = STATE_LEN_LOW;
        }
        break;

    case STATE_LEN_LOW:
        RxBuffer[RxIndex++] = byte;
        DataLen = byte;
        State = STATE_LEN_HIGH;
        break;

    case STATE_LEN_HIGH:
        RxBuffer[RxIndex++] = byte;
        DataLen |= (byte << 8);
        // 完整帧长度 = 5字节帧头 + 2字节命令码 + data + 2字节CRC16。
        if (DataLen <= (sizeof(RxBuffer) - 9))
        {
            State = STATE_SEQ;
        }
        else
        {
            State = STATE_SOF;
        }
        break;

    case STATE_SEQ:
        RxBuffer[RxIndex++] = byte;
        State = STATE_CRC8;
        break;

    case STATE_CRC8:
        RxBuffer[RxIndex++] = byte;
        // 校验帧头CRC8
        if (Verify_CRC8_Check_Sum(RxBuffer, 5))
        {
            State = STATE_CMD_ID_LOW;
        }
        else
        {
            State = STATE_SOF; // 校验失败，重新寻找帧头
        }
        break;

    case STATE_CMD_ID_LOW:
        RxBuffer[RxIndex++] = byte;
        State = STATE_CMD_ID_HIGH;
        break;

    case STATE_CMD_ID_HIGH:
        RxBuffer[RxIndex++] = byte;
        State = STATE_DATA;
        // 如果数据长度为0 (虽然很少见)，直接跳到尾部
        if (DataLen == 0)
        {
            State = STATE_TAIL_LOW;
        }
        break;

    case STATE_DATA:
        if (RxIndex < (sizeof(RxBuffer) - 2)) // 预留2字节给CRC16
        {
            RxBuffer[RxIndex++] = byte;
            // 头部5 + CmdID 2 + DataLen = 目前期望的索引位置
            if (RxIndex >= (5 + 2 + DataLen))
            {
                State = STATE_TAIL_LOW;
            }
        }
        else
        {
            State = STATE_SOF; // 缓冲区溢出保护
        }
        break;

    case STATE_TAIL_LOW:
        RxBuffer[RxIndex++] = byte;
        State = STATE_TAIL_HIGH;
        break;

    case STATE_TAIL_HIGH:
        RxBuffer[RxIndex++] = byte;
        // 接收完毕，整包校验
        if (Verify_CRC16_Check_Sum(RxBuffer, RxIndex))
        {
            RM_RefereeSystemParseData(RxBuffer, RxIndex);
        }
        State = STATE_SOF;
        break;
        
    default:
        State = STATE_SOF;
        break;
    }
}
// 断连
bool RM_RefereeSystemDir()
{
    // 先读取ISR更新时间，再取当前时间，避免中断更新后相减产生下溢。
    const uint32_t status_tick = referee_status_tick;
    const uint32_t heat_tick = referee_heat_tick;
    const uint32_t now = HAL_GetTick();
    RM_RefereeSystemDirFlag = !referee_status_received || !referee_heat_received ||
        static_cast<uint32_t>(now - status_tick) >= REFEREE_TIMEOUT_MS ||
        static_cast<uint32_t>(now - heat_tick) >= REFEREE_TIMEOUT_MS;
    // 数据过期不代表RX DMA已经停止；仅在接收空闲时重启。
    if (RM_RefereeSystemDirFlag &&
        HAL::UART::get_uart_bus_instance().get_uart1().get_handle()->RxState == HAL_UART_STATE_READY)
        RM_RefereeSystemInit();
    return RM_RefereeSystemDirFlag;
}
// 解析（逐字节，兼容旧接口）
void RM_RefereeSystemParse(const uint8_t *data)
{
    if (data == nullptr) return;
    RM_RefereeSystemGetData(*data);
}
// 解析（批量，DMA+空闲模式用）
void RM_RefereeSystemParse(const uint8_t *data, uint16_t size)
{
    if (data == nullptr || size == 0) return;
    for (uint16_t i = 0; i < size; i++)
    {
        RM_RefereeSystemGetData(data[i]);
    }
}
} // namespace RM_RefereeSystem
