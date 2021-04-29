/******************************
 @file TDT_Devce\inc\LCD.h
 @brief  LCD��Ļ����
 @author ����
 @version 2.3.0
 @date 20.12.19
 @history:
	����������������������������������������������������������������������������������������������������������������������������������������������������
	20.12.19 �״����
	����������������������������������������������������������������������������������������������������������������������������������������������������
*****************************/
#ifndef _LCD_H
#define _LCD_H

#include "board.h"
#include "spi.h"

namespace Lcd
{
    enum lcdDirection //Ĭ����Ļ���볯���� ��˳��Ϊ��Ļ˳ʱ����ת0�㡢90�㡢180�㡢270��
    {
        up,
        right,
        down,
        left
    };

    enum Color : uint16_t //16λ�߲�ɫ������R5B5G5A1��ʽ
    {
        WHITE = 0xFFFF,
        BLACK = 0x0000,
        BLUE = 0x001F,
        BRED = 0XF81F,
        GRED = 0XFFE0,
        GBLUE = 0X07FF,
        RED = 0xF800,
        MAGENTA = 0xF81F,
        GREEN = 0x07E0,
        CYAN = 0x7FFF,
        YELLOW = 0xFFE0,
        BROWN = 0XBC40,      //��ɫ
        BRRED = 0XFC07,      //�غ�ɫ
        GRAY = 0X8430,       //��ɫ
        DARKBLUE = 0X01CF,   //����ɫ
        LIGHTBLUE = 0X7D7C,  //ǳ��ɫ
        GRAYBLUE = 0X5458,   //����ɫ
        LIGHTGREEN = 0X841F, //ǳ��ɫ
        LIGHTGRAY = 0XEF5B,  //ǳ��ɫ(PANNEL)
        LGRAY = 0XC618,      //ǳ��ɫ(PANNEL),
        LGRAYBLUE = 0XA651,  //ǳ����ɫ(�м����ɫ)
        LBBLUE = 0X2B12      //ǳ����ɫ(ѡ����Ŀ��0��ɫ)
    };

    enum Command : uint8_t // ILI9341оƬ���δȫ���г�
    {
        nopCommand = 0x00,              //��ָ��
        softwareReset = 0x01,           //�����λ
        enterSleep = 0x10,              //����˯��ģʽ
        exitSleep = 0x11,               //�˳�˯��ģʽ
        partialMode = 0x12,             //������ʾģʽ
        nomalMode = 0x13,               //������ʾģʽ
        displayInversionOn = 0x20,      //��ɫ��ʾ
        displayInversionOff = 0x20,     //ȡ����ɫ��ʾ
        gammaSet = 0x26,                //�趨٤��ֵ
        displayOff = 0x28,              //�ر���ʾ������Ļ��Ϊ����
        displayOn = 0x29,               //����ʾ��
        setX = 0x2A,                    //�趨x����ʾ��ַ
        setY = 0x2B,                    //�趨y����ʾ��ַ
        writeRAM = 0x2C,                //��ʼд�뻺��
        setColor = 0x2D,                //������ɫ��ʽ
        partialArea = 0x30,             //�趨��������
        verticalScrolling = 0x33,       //���ô�ֱ����
        displayFunctionControl = 0xB6,  //��ʾ���ܿ���
        powerControl1 = 0xC0,           //���ʿ���0
        powerControl2 = 0xC1,           //���ʿ���1
        vcomControl1 = 0xC5,            //VCOM����1
        memoryAccessControl = 0x36,     //��Ļ�������
        pixelFormatSet = 0x3A,          //���ظ�ʽ����
        vcomControl2 = 0xC7,            //VCOM����2
        powerControlA = 0xCB,           //���ʿ���A
        powerControlB = 0xCF,           //���ʿ���B
        positiveGammaCorrection = 0xE0, //����٤��У׼
        negativeGammaCorrection = 0xE1, //����٤��У׼
        diverTimingControlA = 0xE8,     //����ʱ�����A
        diverTimingControlB = 0xEA,     //����ʱ�����B
        powerSequenceControl = 0xED,    //��Դ���п���
        enable3Gamma = 0xF2,            //ʹ��٤��
        pumpRatioControl = 0xF7,        //�ñȿ���

    };

    enum fontLibrary //�ֿ�
    {
        ascii1608 = 16,
        ascii2412 = 24,
        ascii3216 = 32,
        ch16x16 = 16,
        ch24x24 = 24,
        ch32x32 = 32,
    };

    enum fontSize //�ֺ�
    {
        small = 16,
        middle = 24,
        large = 32,
    };

    const int16_t LcdWidth = 240; //��Ļ���
    const int16_t LcdHight = 320; //��Ļ����

} // namespace Lcd

struct Pin // �����˿ں�����
{
    GPIO_TypeDef *port;
    uint16_t pin;
};

class LCD : public Spi
{
private:
    Pin dc;    //����/��������ź�
    Pin reset; //lcd��λ�ź�

    uint16_t width;                   //LCD ���
    uint16_t height;                  //LCD �߶�
    enum Lcd::lcdDirection direction; //��Ļ����

    enum Lcd::Color fontColor; //������ɫ
    enum Lcd::Color backColor; //������ɫ

    void lcdPinInit(void); //lcd�����ߺ͸�λ�����ų�ʼ��

    inline void lcdDcSet(void);      //lcd����������
    inline void lcdDcClear(void);    //lcd����������
    inline void lcdResetSet(void);   //lcd��λ������
    inline void lcdResetClear(void); //lcd��λ������

    inline void lcdSendByte(uint8_t data); //��spi���ͺ������ж��η�װ

    void lcdSendCommand(uint8_t data);    //��������
    void lcdSendData(uint8_t data);       //��������
    void lcdSendData16Bit(uint16_t data); //����16λ����
    void lcdWriteRAMPrepare(void);        //��ʼд�뻺��

    void lcdDrawCircle8(int xc, int yc, int x, int y, enum Lcd::Color color);                                //8��Գƻ�Բ�㷨
    void lcdDrawFill(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color); //�������

    void lcdShowChar(uint16_t x, uint16_t y, char cha, enum Lcd::fontLibrary size, enum Lcd::Color fontColor, enum Lcd::Color backColor, uint8_t mode); //��ʾ�ַ�
    void lcdShowCH(uint16_t x, uint16_t y, char *ch, enum Lcd::fontLibrary size, enum Lcd::Color fontColor, enum Lcd::Color backColor, uint8_t mode);   //��ʾ���ĺ���

public:
    LCD(GPIO_TypeDef *dcPort, uint16_t dcPin, GPIO_TypeDef *resetPort, uint16_t resetPin, SPI_TypeDef *spix);

    void lcdInit(void);                   //LCD��Ļ��ʼ��
    void lcdReset(void);                  //��λlcd��
    void lcdClear(enum Lcd::Color color); //����

    void lcdSetDirection(enum Lcd::lcdDirection direction);                             //������Ļ����
    void lcdSetWindows(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd); //�趨lcd��Ļ��������
    void lcdSetCursor(uint16_t xPos, uint16_t yPos);                                    //�趨���
    void lcdSetColor(enum Lcd::Color fontColor, enum Lcd::Color backColor);             //������ʾ�ַ�����ɫ

    void lcdDrawPoint(uint16_t x, uint16_t y, enum Lcd::Color color);                                                              //��һ����
    void lcdDrawLine(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color);                       //��һ����
    void lcdDrawRectangle(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color);                  //���ƿ��ľ���
    void lcdDrawFillRectangle(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color);              //����ʵ�ľ���
    void lcdDrawCircle(uint16_t x, uint16_t y, uint16_t r, uint8_t fill, enum Lcd::Color color);                                   //��Բ
    void lcdDrawTriangel(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, enum Lcd::Color color);     //���ƿ���������
    void lcdDrawFillTriangel(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, enum Lcd::Color color); //����ʵ��������

    //��ʾ�ַ���
    void lcdShowString(uint16_t x, uint16_t y, char *str, enum Lcd::fontSize size, enum Lcd::Color fontColor, uint8_t mode = 1, enum Lcd::Color backColor = Lcd::BLACK);
    //��ʽ����ӡ�ַ���
    void lcdPrintf(uint16_t x, uint16_t y, enum Lcd::fontSize size, uint8_t mode, const char *format, ...);
};

#endif
