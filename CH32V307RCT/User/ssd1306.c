#include "ssd1306.h"
#include "font.h"
#include "debug.h"

// 128x64 的显存，每个字节代表8个像素
static uint8_t OLED_GRAM[8][128];

// 简单的I2C写一个字节函数 (基于CH32V307硬件I2C)
void I2C_WriteByte(uint8_t addr, uint8_t data) {
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, data);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(I2C1, ENABLE);
}

void SSD1306_WriteCmd(uint8_t cmd) {
    I2C_WriteByte(SSD1306_I2C_ADDR, 0x00); // 写命令
    I2C_WriteByte(SSD1306_I2C_ADDR, cmd);
}

void SSD1306_WriteData(uint8_t data) {
    I2C_WriteByte(SSD1306_I2C_ADDR, 0x40); // 写数据
    I2C_WriteByte(SSD1306_I2C_ADDR, data);
}

void SSD1306_Init(void) {
    Delay_Ms(100);
    SSD1306_WriteCmd(0xAE); // 关闭显示
    SSD1306_WriteCmd(0x20); // 设置内存寻址模式
    SSD1306_WriteCmd(0x10); // 00,水平寻址;01,垂直寻址;10,页寻址(默认)
    SSD1306_WriteCmd(0xB0); // 为页寻址模式设置起始页寄存器
    SSD1306_WriteCmd(0xC8); // 设置COM输出扫描方向
    SSD1306_WriteCmd(0x00); // 设置低列地址
    SSD1306_WriteCmd(0x10); // 设置高列地址
    SSD1306_WriteCmd(0x40); // 设置起始行地址
    SSD1306_WriteCmd(0x81); // 设置对比度控制寄存器
    SSD1306_WriteCmd(0xFF); // 亮度调节 0x00~0xFF
    SSD1306_WriteCmd(0xA1); // 段重定向设置
    SSD1306_WriteCmd(0xA6); // 正常显示
    SSD1306_WriteCmd(0xA8); // 设置多路复用率(1 to 64)
    SSD1306_WriteCmd(0x3F); // 1/64 duty
    SSD1306_WriteCmd(0xA4); // 全局显示开启
    SSD1306_WriteCmd(0xD3); // 设置显示偏移
    SSD1306_WriteCmd(0x00); // 偏移量为0
    SSD1306_WriteCmd(0xD5); // 设置显示时钟分频比/振荡器频率
    SSD1306_WriteCmd(0xF0); // 设置分频比
    SSD1306_WriteCmd(0xD9); // 设置预充电周期
    SSD1306_WriteCmd(0x22); // 
    SSD1306_WriteCmd(0xDA); // 设置COM硬件引脚配置
    SSD1306_WriteCmd(0x12); // 
    SSD1306_WriteCmd(0xDB); // 设置vcomh
    SSD1306_WriteCmd(0x20); // 0x20,0.77xVcc
    SSD1306_WriteCmd(0x8D); // 电荷泵设置
    SSD1306_WriteCmd(0x14); // 开启电荷泵
    SSD1306_WriteCmd(0xAF); // 开启OLED显示
    SSD1306_Clear();
    SSD1306_Refresh();
}

void SSD1306_SetCursor(uint8_t x, uint8_t y) {
    SSD1306_WriteCmd(0xB0 + y);
    SSD1306_WriteCmd(((x & 0xF0) >> 4) | 0x10);
    SSD1306_WriteCmd((x & 0x0F) | 0x01);
}

void SSD1306_Clear(void) {
    for(uint8_t i = 0; i < 8; i++) {
        for(uint8_t j = 0; j < 128; j++) {
            OLED_GRAM[i][j] = 0;
        }
    }
}

void SSD1306_ShowChar(uint8_t x, uint8_t y, char chr) {
    uint8_t c = chr - ' '; // 得到偏移后的值
    if(x > 128 - 6 || y > 7) return;
    for(uint8_t i = 0; i < 6; i++) {
        OLED_GRAM[y][x + i] = F6x8[c][i];
    }
}

void SSD1306_ShowString(uint8_t x, uint8_t y, char *str) {
    while(*str) {
        SSD1306_ShowChar(x, y, *str);
        x += 6;
        if(x > 122) { x = 0; y++; }
        str++;
    }
}

void SSD1306_Refresh(void) {
    for(uint8_t i = 0; i < 8; i++) {
        SSD1306_SetCursor(0, i);
        for(uint8_t j = 0; j < 128; j++) {
            SSD1306_WriteData(OLED_GRAM[i][j]);
        }
    }
}