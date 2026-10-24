/**
 * =====================================================================
 * @file    main.c
 * @brief   智能环境监测与排风系统 - 主控程序
 * @author  [邱家锋]、[曾锦程]、[郭乔松]
 * @team    [永雏塔菲]
 * @date    2026-10-24
 * 
 * @功能说明：
 * 1. 软件模拟 I2C 驱动 OLED 屏幕显示数据。
 * 2. 读取 AHT20 传感器获取温湿度（含状态位校验，防数据丢包跳变）。
 * 3. 读取 BMP280 传感器获取大气压强（含官方复杂补偿算法）。
 * 4. 通过 ADC 采集 MQ-2 烟雾传感器模拟量。
 * 5. 逻辑联动：当烟雾浓度超标时，触发蜂鸣器声光报警，并通过继电器开启排风扇。
 * =====================================================================
 */

#include "ch32v30x.h"
#include <stdio.h> 

// ================== I2C 引脚宏定义 ==================
// 采用开漏输出 (Out_OD) 模拟 I2C 总线，允许总线释放，确保多设备正常应答
#define SCL_PIN  GPIO_Pin_6
#define SDA_PIN  GPIO_Pin_7
#define I2C_PORT GPIOB

#define SCL_H()  GPIO_SetBits(I2C_PORT, SCL_PIN)
#define SCL_L()  GPIO_ResetBits(I2C_PORT, SCL_PIN)
#define SDA_H()  GPIO_SetBits(I2C_PORT, SDA_PIN)
#define SDA_L()  GPIO_ResetBits(I2C_PORT, SDA_PIN)
#define SDA_READ GPIO_ReadInputDataBit(I2C_PORT, SDA_PIN)

// ================== 基础延时函数 ==================
void delay_us(uint32_t us) { for(uint32_t i = 0; i < us * 8; i++) { __NOP(); } }
void delay_ms(uint32_t ms) { for(uint32_t i = 0; i < ms; i++) delay_us(1000); }

// ================== 软件 I2C 核心时序 ==================
void I2C_Start(void) { SDA_H(); SCL_H(); delay_us(4); SDA_L(); delay_us(4); SCL_L(); delay_us(4); }
void I2C_Stop(void) { SCL_L(); SDA_L(); delay_us(4); SCL_H(); SDA_H(); delay_us(4); }

uint8_t I2C_SendByte(uint8_t byte) {
    uint8_t i, ack;
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H(); else SDA_L();
        byte <<= 1; delay_us(2); SCL_H(); delay_us(4); SCL_L(); delay_us(4);
    }
    SDA_H(); delay_us(2); SCL_H(); delay_us(4);
    ack = SDA_READ;       
    SCL_L(); delay_us(4);
    return ack;
}

uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t i, byte = 0;
    SDA_H();
    for (i = 0; i < 8; i++) {
        SCL_H(); delay_us(4);
        byte <<= 1;
        if (SDA_READ) byte |= 1;
        SCL_L(); delay_us(4);
    }
    if(ack) SDA_L(); else SDA_H();    
    SCL_H(); delay_us(4); SCL_L(); delay_us(4); SDA_H(); 
    return byte;
}

// ================== OLED 屏幕驱动 ==================
void OLED_WriteCmd(uint8_t cmd) { I2C_Start(); I2C_SendByte(0x78); I2C_SendByte(0x00); I2C_SendByte(cmd); I2C_Stop(); }
void OLED_WriteData(uint8_t data) { I2C_Start(); I2C_SendByte(0x78); I2C_SendByte(0x40); I2C_SendByte(data); I2C_Stop(); }

void OLED_Init(void) {
    delay_ms(100);
    OLED_WriteCmd(0xAE); OLED_WriteCmd(0x20); OLED_WriteCmd(0x10); OLED_WriteCmd(0xB0);
    OLED_WriteCmd(0xC8); OLED_WriteCmd(0x00); OLED_WriteCmd(0x10); OLED_WriteCmd(0x40);
    OLED_WriteCmd(0x81); OLED_WriteCmd(0xFF); OLED_WriteCmd(0xA1); OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F); OLED_WriteCmd(0xA4); OLED_WriteCmd(0xD3);
    OLED_WriteCmd(0x00); OLED_WriteCmd(0xD5); OLED_WriteCmd(0xF0); OLED_WriteCmd(0xD9);
    OLED_WriteCmd(0x22); OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12); OLED_WriteCmd(0xDB);
    OLED_WriteCmd(0x20); OLED_WriteCmd(0x8D); OLED_WriteCmd(0x14); OLED_WriteCmd(0xAF);
}

void OLED_Clear(void) {
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WriteCmd(0xB0 + i); OLED_WriteCmd(0x00); OLED_WriteCmd(0x10);
        for (uint8_t j = 0; j < 128; j++) OLED_WriteData(0x00);
    }
}

// 字库（截取常用字符，节省空间）
const uint8_t F6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x2f,0x00,0x00,0x00},{0x00,0x07,0x00,0x07,0x00,0x00},
    {0x14,0x7f,0x14,0x7f,0x14,0x00},{0x24,0x2a,0x7f,0x2a,0x12,0x00},{0x23,0x13,0x08,0x64,0x62,0x00},
    {0x36,0x49,0x55,0x22,0x50,0x00},{0x00,0x05,0x03,0x00,0x00,0x00},{0x00,0x1c,0x22,0x41,0x00,0x00},
    {0x00,0x41,0x22,0x1c,0x00,0x00},{0x14,0x08,0x3e,0x08,0x14,0x00},{0x08,0x08,0x3e,0x08,0x08,0x00},
    {0x00,0x50,0x30,0x00,0x00,0x00},{0x08,0x08,0x08,0x08,0x08,0x00},{0x00,0x60,0x60,0x00,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02,0x00},{0x3e,0x51,0x49,0x45,0x3e,0x00},{0x00,0x42,0x7f,0x40,0x00,0x00},
    {0x42,0x61,0x51,0x49,0x46,0x00},{0x21,0x41,0x45,0x4b,0x31,0x00},{0x18,0x14,0x12,0x7f,0x10,0x00},
    {0x27,0x45,0x45,0x45,0x39,0x00},{0x3c,0x4a,0x49,0x49,0x30,0x00},{0x01,0x71,0x09,0x05,0x03,0x00},
    {0x36,0x49,0x49,0x49,0x36,0x00},{0x06,0x49,0x49,0x29,0x1e,0x00},{0x00,0x36,0x36,0x00,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00,0x00},{0x08,0x14,0x22,0x41,0x00,0x00},{0x14,0x14,0x14,0x14,0x14,0x00},
    {0x00,0x41,0x22,0x14,0x08,0x00},{0x02,0x01,0x51,0x09,0x06,0x00},{0x32,0x49,0x79,0x41,0x3e,0x00},
    {0x7e,0x11,0x11,0x11,0x7e,0x00},{0x7f,0x49,0x49,0x49,0x36,0x00},{0x3e,0x41,0x41,0x41,0x22,0x00},
    {0x7f,0x41,0x41,0x22,0x1c,0x00},{0x7f,0x49,0x49,0x49,0x41,0x00},{0x7f,0x09,0x09,0x09,0x01,0x00},
    {0x3e,0x41,0x49,0x49,0x7a,0x00},{0x7f,0x08,0x08,0x08,0x7f,0x00},{0x00,0x41,0x7f,0x41,0x00,0x00},
    {0x20,0x40,0x41,0x3f,0x01,0x00},{0x7f,0x08,0x14,0x22,0x41,0x00},{0x7f,0x40,0x40,0x40,0x40,0x00},
    {0x7f,0x02,0x0c,0x02,0x7f,0x00},{0x7f,0x04,0x08,0x10,0x7f,0x00},{0x3e,0x41,0x41,0x41,0x3e,0x00},
    {0x7f,0x09,0x09,0x09,0x06,0x00},{0x3e,0x41,0x51,0x21,0x5e,0x00},{0x7f,0x09,0x19,0x29,0x46,0x00},
    {0x46,0x49,0x49,0x49,0x31,0x00},{0x01,0x01,0x7f,0x01,0x01,0x00},{0x3f,0x40,0x40,0x40,0x3f,0x00},
    {0x1f,0x20,0x40,0x20,0x1f,0x00},{0x3f,0x40,0x38,0x40,0x3f,0x00},{0x63,0x14,0x08,0x14,0x63,0x00},
    {0x07,0x08,0x70,0x08,0x07,0x00},{0x61,0x51,0x49,0x45,0x43,0x00},{0x00,0x7f,0x41,0x41,0x00,0x00},
    {0x02,0x04,0x08,0x10,0x20,0x00},{0x00,0x41,0x41,0x7f,0x00,0x00},{0x04,0x02,0x01,0x02,0x04,0x00},
    {0x40,0x40,0x40,0x40,0x40,0x00},{0x00,0x01,0x02,0x04,0x00,0x00},{0x20,0x54,0x54,0x54,0x78,0x00},
    {0x7f,0x48,0x44,0x44,0x38,0x00},{0x38,0x44,0x44,0x44,0x20,0x00},{0x38,0x44,0x44,0x48,0x7f,0x00},
    {0x38,0x54,0x54,0x54,0x18,0x00},{0x08,0x7e,0x09,0x01,0x02,0x00},{0x0c,0x52,0x52,0x52,0x3e,0x00},
    {0x7f,0x08,0x04,0x04,0x78,0x00},{0x00,0x44,0x7d,0x40,0x00,0x00},{0x20,0x40,0x44,0x3d,0x00,0x00},
    {0x7f,0x10,0x28,0x44,0x00,0x00},{0x00,0x41,0x7f,0x40,0x00,0x00},{0x7c,0x04,0x18,0x04,0x78,0x00},
    {0x7c,0x08,0x04,0x04,0x78,0x00},{0x38,0x44,0x44,0x44,0x38,0x00},{0x7c,0x14,0x14,0x14,0x08,0x00},
    {0x08,0x14,0x14,0x18,0x7c,0x00},{0x7c,0x08,0x04,0x04,0x08,0x00},{0x48,0x54,0x54,0x54,0x20,0x00},
    {0x04,0x3f,0x44,0x40,0x20,0x00},{0x3c,0x40,0x40,0x20,0x7c,0x00},{0x1c,0x20,0x40,0x20,0x1c,0x00},
    {0x3c,0x40,0x30,0x40,0x3c,0x00},{0x44,0x28,0x10,0x28,0x44,0x00},{0x0c,0x50,0x50,0x50,0x3c,0x00},
    {0x44,0x64,0x54,0x4c,0x44,0x00},{0x00,0x08,0x36,0x41,0x00,0x00},{0x00,0x00,0x7f,0x00,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00,0x00},{0x08,0x04,0x08,0x10,0x08,0x00}
};

void OLED_ShowChar(uint8_t x, uint8_t y, char chr) {
    if(chr < ' ' || chr > '~') return;
    uint8_t c = chr - ' ';
    OLED_WriteCmd(0xB0 + y); OLED_WriteCmd(((x & 0xF0) >> 4) | 0x10); OLED_WriteCmd((x & 0x0F) | 0x01);
    for (uint8_t i = 0; i < 6; i++) OLED_WriteData(F6x8[c][i]);
}

void OLED_ShowString(uint8_t x, uint8_t y, char *str) {
    while (*str) { OLED_ShowChar(x, y, *str); x += 6; if (x > 122) { x = 0; y++; } str++; }
}

// ================== AHT20 温湿度驱动 ==================
uint8_t AHT_Addr = 0x70;
uint8_t AHT_Read = 0x71;

// 自动探测 I2C 地址并初始化
uint8_t AHT20_Init(void) {
    delay_ms(50); 
    I2C_Start();
    if(I2C_SendByte(0x70) == 0) { AHT_Addr = 0x70; AHT_Read = 0x71; }
    else {
        I2C_Start();
        if(I2C_SendByte(0x71) == 0) { AHT_Addr = 0x71; AHT_Read = 0x70; }
        else { I2C_Stop(); return 1; }
    }
    I2C_SendByte(0xA8); I2C_SendByte(0x00); I2C_SendByte(0x00);
    I2C_Stop();
    delay_ms(100); // 等待内部校准
    return 0; 
}

// 带有状态位拦截的严谨读取
void AHT20_Read(float *temp, float *humi) {
    uint8_t buf[7] = {0};
    
    I2C_Start(); I2C_SendByte(AHT_Addr); I2C_SendByte(0xAC); I2C_SendByte(0x33); I2C_SendByte(0x00); I2C_Stop();
    delay_ms(100); 
    
    I2C_Start(); I2C_SendByte(AHT_Read); 
    for(int i=0; i<7; i++) { buf[i] = I2C_ReadByte((i==6) ? 0 : 1); }
    I2C_Stop();
    
    // 校验：Bit7 为忙标志，Bit3 为校准标志。若异常直接返回，防止屏幕显示乱码
    if((buf[0] & 0x80) != 0 || (buf[0] & 0x08) == 0) return; 
    
    uint32_t hum_raw = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t temp_raw = ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    
    *humi = (float)hum_raw * 100.0 / (1 << 20);
    *temp = (float)temp_raw * 200.0 / (1 << 20) - 50.0;
}

// ================== BMP280 气压驱动 ==================
uint8_t BMP_Addr = 0xEC;
uint8_t BMP_Read = 0xED;
uint16_t dig_T1; int16_t dig_T2, dig_T3;
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

uint8_t BMP280_Init(void) {
    I2C_Start();
    if(I2C_SendByte(0xEC) == 0) { BMP_Addr = 0xEC; BMP_Read = 0xED; }
    else {
        I2C_Start();
        if(I2C_SendByte(0xEE) == 0) { BMP_Addr = 0xEE; BMP_Read = 0xEF; }
        else { I2C_Stop(); return 1; }
    }
    
    // 读取官方校准参数
    I2C_SendByte(0x88); I2C_Stop();
    I2C_Start(); I2C_SendByte(BMP_Read);
    dig_T1 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8); dig_T2 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8); dig_T3 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8);
    dig_P1 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8); dig_P2 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8); dig_P3 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8);
    dig_P4 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8); dig_P5 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8); dig_P6 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8);
    dig_P7 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8); dig_P8 = I2C_ReadByte(1) | (I2C_ReadByte(1)<<8); dig_P9 = I2C_ReadByte(1) | (I2C_ReadByte(0)<<8);
    I2C_Stop();
    
    I2C_Start(); I2C_SendByte(BMP_Addr); I2C_SendByte(0xF4); I2C_SendByte(0x27); I2C_Stop();
    delay_ms(10);
    return 0; 
}

// 包含官方的复杂补偿算法
float BMP280_Read_Pressure(void) {
    int32_t adc_P, adc_T; int32_t var1, var2; int32_t p;
    I2C_Start(); I2C_SendByte(BMP_Addr); I2C_SendByte(0xF7); I2C_Stop();
    I2C_Start(); I2C_SendByte(BMP_Read);
    adc_P = (int32_t)I2C_ReadByte(1) << 12; adc_P |= (int32_t)I2C_ReadByte(1) << 4; adc_P |= (int32_t)I2C_ReadByte(1) >> 4;
    adc_T = (int32_t)I2C_ReadByte(1) << 12; adc_T |= (int32_t)I2C_ReadByte(1) << 4; adc_T |= (int32_t)I2C_ReadByte(0) >> 4;
    I2C_Stop();
    
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
    if (var1 == 0) return 0; 
    
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) p = (p << 1) / ((uint32_t)var1);
    else p = (p / (uint32_t)var1) * 2;
    var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
    p = (int32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
    
    return (float)p / 100.0; 
}

// ================== 硬件初始化 (ADC 与 报警执行器) ==================
void Hardware_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); 

    // PA0：MQ-2 模拟输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA1：蜂鸣器，PA2：继电器
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2); 

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);

    // ADC 必须校准，否则数据漂移严重
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

uint16_t Get_ADC_Value(void) {
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}

// ================== 主程序入口 ==================
int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_ms(100); 

    // 配置 I2C 引脚为开漏输出
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);
    SCL_H(); SDA_H();
    
    OLED_Init();
    OLED_Clear();
    Hardware_Init();

    char buf[20];
    uint16_t air_quality = 0;
    float temp = 25.0, humi = 50.0, press = 1013.0; // 初始基准值
    
    // 开机自检提示
    OLED_ShowString(0, 0, "System Running...");
    AHT20_Init(); 
    BMP280_Init(); 
    delay_ms(1000);
    OLED_Clear();

    while (1) {
        // 1. 传感器数据采集
        air_quality = Get_ADC_Value();
        AHT20_Read(&temp, &humi);
        press = BMP280_Read_Pressure();

        // 2. 数据格式化（规避 printf 浮点 bug，拆分为整数显示）
        int t_int = (int)temp;                   
        int t_dec = (int)((temp - t_int) * 10);              
        int h_int = (int)humi;                   
        int h_dec = (int)((humi - h_int) * 10);  
        
        // 3. OLED 界面刷新
        OLED_ShowString(0, 0, "Temp: ");  sprintf(buf, "%d.%d C   ", t_int, t_dec); OLED_ShowString(36, 0, buf);
        OLED_ShowString(0, 2, "Humi: ");  sprintf(buf, "%d.%d %%   ", h_int, h_dec); OLED_ShowString(36, 2, buf);
        OLED_ShowString(0, 4, "Pres: ");  sprintf(buf, "%d hPa  ", (int)press); OLED_ShowString(36, 4, buf);
        OLED_ShowString(0, 6, "Air: ");   sprintf(buf, "%d   ", air_quality); OLED_ShowString(30, 6, buf);

        // 4. 报警与排风联动控制 (设定安全阈值 1500)
        if (air_quality > 1500) {
            OLED_ShowString(75, 6, "ALARM!"); 
            GPIO_SetBits(GPIOA, GPIO_Pin_1);   // 开启蜂鸣器
            GPIO_SetBits(GPIOA, GPIO_Pin_2);   // 继电器吸合，开启风扇
        } else {
            OLED_ShowString(75, 6, "      ");  // 清除报警字样
            GPIO_ResetBits(GPIOA, GPIO_Pin_1); // 关闭蜂鸣器
            GPIO_ResetBits(GPIOA, GPIO_Pin_2); // 继电器断开，关闭风扇
        }
        
        delay_ms(500);
    }
}