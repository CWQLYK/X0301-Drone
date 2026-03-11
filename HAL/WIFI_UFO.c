// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期：2026.3.8
// 版本：1.0
//  WIFI_UFO通信协议模块
#include "ALL_DEFINE.h"

// WIFI接收缓冲区（协议帧缓存）
static uint8_t wifi_RxBuffer[8];
// WIFI通信状态标志（0=正常，非0=异常/失联）
uint16_t WIFI_UFO_Err;
// WIFI信号强度相关变量
uint8_t WIFI_SSI,WIFI_SSI_CNT;

// WIFI功能标志位（3D模式等）
static uint8_t wifi_PUI=0;

/**************************************************************
 * @brief  WIFI_UFO数据接收预处理（帧头帧尾校验）
 * @param  data: 单字节接收数据
 * @retval 无
 * @note   1. 协议帧格式：0x66(帧头) + 7字节数据 + 0x99(帧尾)
 *         2. 接收到完整帧后设置WIFI_UFO_Err=0表示数据有效
 ***************************************************************/
void WIFI_UFO_Data_Receive_Prepare(uint8_t data)
{
    static uint8_t _data_cnt = 0;  // 接收字节计数
    static uint8_t state = 0;      // 帧解析状态机

    // 状态0：等待帧头0x66
    if(state==0 && data==0x66)
    {
        state=1;                   // 进入数据接收状态
        wifi_RxBuffer[0]=data;     // 保存帧头
    }
    // 状态1：接收数据字节
    else if(state==1)
    {
        wifi_RxBuffer[_data_cnt++]=data; // 存储数据字节
    }

    // 接收满7字节（含帧尾）
    if(_data_cnt==7)
    {
        state = 0;                 // 复位状态机
        _data_cnt = 0;             // 清零计数
        // 校验帧尾0x99，有效则标记通信正常
        if(data == 0x99) WIFI_UFO_Err=0;
    }
}

/**************************************************************
 * @brief  WIFI模块按键功能处理（基于AUX2通道值）
 * @param  key: Remote.AUX2通道值
 * @retval 无
 * @note   1. 仅在AUX2值变化时执行，防止重复触发
 *         2. 不同AUX2值对应不同功能（3D模式/无头模式/一键停止等）
 ***************************************************************/
static void Key_Function(u8 key)
{
    static u8 temp;  // 上一次AUX2值（防抖用）

    // AUX2值变化时才执行功能逻辑
    if(temp != Remote.AUX2)
    {
        switch(temp)
        {
            case 1: // 一档开关  手机遥控模式生效
                // 预留自动起飞逻辑
                // if(fly_ready==0)
                // {
                //     fly_ready = 1;
                //     high_start = HIGH_START;
                // }
                break;
            case 2: // 二档开关  手机遥控模式生效
                // 预留自动降落逻辑
                // if(fly_ready==1)
                // {
                //     auto_landing = 1;
                // }
                break;
            case 8: // 3D模式（一档开关值8，关闭时为0）
                wifi_PUI = 1;
                break;
            case 16: // 无头模式（一档开关值16，关闭时为0）
                break;
            case 128: // 一键停止（拨杆第三次切换值为128）
                break;
            default:
                wifi_PUI = 0; // 关闭3D模式
                break;
        }
        temp = Remote.AUX2; // 更新上一次AUX2值
    }
}

// WIFI接收缓冲区2（备用缓存）
static u8 wifi_RxBuffer22[8];

/**************************************************************
 * @brief  WIFI_UFO数据解析（APP遥控数据转遥控器通道值）
 * @param  data_buf: 接收数据缓冲区指针
 * @param  num: 数据长度（固定8字节）
 * @retval 无
 * @note   1. 协议帧示例：66 80 80 00 80 00 80 99
 *         2. 将0-255范围的APP数据转换为1000-2000的标准遥控器通道值
 ***************************************************************/
void WIFI_UFO_Data_Receive_Anl(u8 *data_buf,u8 num)
{
    // 设置控制来源标志：1=遥控器 2=WiFi图传模块 3=其他模块
    flag.NS = 2;

    // ------------------------ 遥控器通道值转换 ------------------------
    // 油门通道：0-255 → 1000-2000
    Remote.thr = ((float)*(data_buf+2)/256) * 1000 + 1000;
    LIMIT(Remote.thr,1000,2000); // 限幅防止超范围

    // 偏航通道：0-255 → 1000-2000（系数3.90625=1000/256）
    Remote.yaw  = ((float)*(data_buf+3)*3.90625) + 1000;
    LIMIT(Remote.yaw,1000,2000);

    // 横滚通道：0-255 → 1000-2000
    Remote.roll = (((float)*(data_buf+0)/256)*1000) + 1000;
    LIMIT(Remote.roll,1000,2000);

    // 俯仰通道：0-255 → 1000-2000
    Remote.pitch = (((float)*(data_buf+1)/256)*1000)+ 1000;
    LIMIT(Remote.pitch,1000,2000);

    // ------------------------ 原始数据缓存 ------------------------
    wifi_RxBuffer22[0] = ((float)*(data_buf+0));
    wifi_RxBuffer22[1] = ((float)*(data_buf+1));
    wifi_RxBuffer22[2] = ((float)*(data_buf+2));
    wifi_RxBuffer22[3] = ((float)*(data_buf+3));
    wifi_RxBuffer22[4] = ((float)*(data_buf+4));
    wifi_RxBuffer22[5] = ((float)*(data_buf+5));
    wifi_RxBuffer22[6] = ((float)*(data_buf+6));
    wifi_RxBuffer22[7] = ((float)*(data_buf+7));

    // ------------------------ 辅助通道处理 ------------------------
    // AUX2通道值（功能按键）
    Remote.AUX2 = *(data_buf+4);
    // 执行按键功能逻辑
    Key_Function(Remote.AUX2);
}

/**************************************************************
 * @brief  WIFI_UFO连接状态检测
 * @param  无
 * @retval u8: 连接标志（1=已连接，0=未连接）
 * @note   1. 通过WIFI_UFO_Err计数判断是否失联（超过30次判定为失联）
 *         2. 连接正常时周期性解析数据并打印调试信息
 ***************************************************************/
u8 WIFI_UFO_Connect(void)
{
    static u8 Connect_flag; // 连接状态标志
    static u8 count10=0;    // 调试打印计数

    WIFI_UFO_Err ++; // 失联计数累加

    // 有有效数据接收时（WIFI_UFO_Err==1）
    if(WIFI_UFO_Err==1)
    {
        // 解析WIFI接收数据
        WIFI_UFO_Data_Receive_Anl(wifi_RxBuffer,8);
        // 信号强度计数
        WIFI_SSI_CNT++;
        // 标记已连接
        Connect_flag = 1;
        // 调试打印计数
        count10++;

        // 每2次解析打印一次调试信息
        if(count10>=2)
        {
            count10=0;
            // 3D模式下打印遥控器通道值
            if(wifi_PUI==1)
            {
                // printf("Remote.thr: %d  Remote.yaw: %d  Remote.roll: %d   Remote.pitch: %d  \r\n",Remote.thr,Remote.yaw,Remote.roll,Remote.pitch);
            }
            // 打印原始接收数据
            // printf("rxbuf0: %d  rxbuf1: %d  rxbuf2: %d  rxbuf3: %d   rxbuf4: %d  rxbuf5: %d  rxbuf6: %d   rxbuf7: %d   \r\n",wifi_RxBuffer22[0],wifi_RxBuffer22[1],wifi_RxBuffer22[2],wifi_RxBuffer22[3],wifi_RxBuffer22[4],wifi_RxBuffer22[5],wifi_RxBuffer22[6],wifi_RxBuffer22[7]);
        }
    }

    // 失联处理：累计30次无有效数据判定为失联
    if(WIFI_UFO_Err>=30)
    {
        WIFI_UFO_Err = 1;       // 复位失联计数
        Connect_flag = 0;       // 标记未连接
        flag.NS = 0;            // 清空控制来源标志
        // printf("wifi失联 \r\n");
    }

    return Connect_flag; // 返回连接状态
}


