#include "stm32f4xx.h"                  // Device header
#include "dma.h"
#include "usart.h"
//#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "joint.h"
//joint1->huart2
//joint2->huart4
//joint3->huart7
//joint4->huart8
// 全局变量
UART_HandleTypeDef * joint_uart[3]={&huart2,&huart4,&huart7};
#define MAX_JOINTS 4  // 根据实际关节数量修改
// 错误代码定义
#define JOINT_SUCCESS             0
#define JOINT_ERROR_INVALID_ID   -1
#define JOINT_ERROR_HW_INIT      -2
#define JOINT_ERROR_CONFIG       -3
#define JOINT_ERROR_POWER        -4
#define JOINT_ERROR_COMM         -5
#define JOINT_ERROR_TIMEOUT      -6
//单片机→驱动器，指令报文格式
//使能驱动器
uint8_t Start1[11] = {0x53,0x09,0x01,0x02,0x40,0x60,0x00,0x06,0x00,0x2C,0x45};
uint8_t Start2[11] = {0x53,0x09,0x01,0x02,0x40,0x60,0x00,0x0F,0x00,0x25,0x45};
uint8_t plus_mode[9] = {0x53,0x07,0x01,0x01,0x60,0x60,0x00,0x52,0x45};
uint8_t plus_mode1[10] = {0x53,0x08,0x01,0x02,0x60,0x60,0x00,0x01,0xF5,0x45};

uint8_t target_mode[10] = {0x53,0x08,0x01,0x02,0x60,0x60,0x00,0x08,0xFC,0x45};
uint8_t homing_mode[10] = {0x53,0x08,0x01,0x02,0x60,0x60,0x00,0x06,0xF2,0x45};
uint8_t homing_method[10] = {0x53,0x08,0x01,0x02,0x98,0x60,0x00,0xFD,0x5B,0x45};
uint8_t homing_method1[10] = {0x53,0x08,0x01,0x02,0x98,0x60,0x00,0xFC,0xA5,0x45};//14:19:50.616	1 SOBJ 6098.00 FD (-3)	Transmit: 53 08 01 02 98 60 00 FD 5B 45	Homing method
uint8_t Telegram1[13] = {0x53,0x0B,0x01,0x02,0x7A,0x60,0x00,0x00,0x00,0x00,0x00,0x47,0x45};
uint8_t Telegram2[13] = {0x53,0x0B,0x01,0x02,0x7A,0x60,0x00,0x00,0x00,0x00,0x00,0x47,0x45};
uint8_t Telegram3[13] = {0x53,0x0B,0x01,0x02,0x7A,0x60,0x00,0x00,0x00,0x00,0x00,0x47,0x45};
uint8_t Telegram4[13] = {0x53,0x0B,0x01,0x02,0x7A,0x60,0x00,0x00,0x00,0x00,0x00,0x47,0x45};
uint8_t Controlword1[11] = {0x53,0x09,0x01,0x02,0x40,0x60,0x00,0x0F,0x00,0x25,0x45};
uint8_t Controlword2[11] = {0x53,0x09,0x01,0x02,0x40,0x60,0x00,0x7F,0x00,0x55,0x45};
uint8_t Controlword3[11] = {0x53,0x09,0x01,0x02,0x40,0x60,0x00,0x1F,0x00,0xCA,0x45};
uint8_t Controlword4[11] = {0x53,0x09,0x01,0x02,0x40,0x60,0x00,0x3F,0x00,0xBF,0x45};
uint8_t Controlword5[11] = {0x53,0x09,0x01,0x02,0x40,0x60,0x00,0x3F,0x00,0xBF,0x45};
uint8_t Position_actual_value[9] = {0x53,0x07,0x01,0x01,0x64,0x60,0x00,0x56,0x45};
uint8_t Position_actual_value1[9] = {0x45};
//uint8_t joint1_RxPacket[10];
extern uint8_t joint1_RxPacket[13];
uint8_t joint2_RxPacket[13];
uint8_t joint3_RxPacket[13];
uint8_t joint4_RxPacket[13];
uint8_t CalcCRCByte(uint8_t u8Byte, uint8_t u8CRC)
{
	uint8_t i;
	uint8_t Polynom = 0xD5;
    u8CRC = u8CRC ^ u8Byte;
    for (i = 0; i < 8; i++) 
	{
		if (u8CRC & 0x01)
        {
			u8CRC = (u8CRC >> 1) ^ Polynom;
        }
        else
		{
            u8CRC >>= 1;
        }
    }
    return u8CRC;
}
uint8_t CalcCRC(uint8_t *Array,uint16_t Length)
{
	uint16_t i;
	uint8_t CRC8[Length+1];
	CRC8[0] = 0xFF;
	for (i = 0; i < Length; i ++)
	{
		CRC8[i+1] = CalcCRCByte(Array[i+1],CRC8[i]);
	}
	return CRC8[Length];
}
int joint_init(int joint_id){
	    // 1. 参数有效性检查
		HAL_UART_Transmit(joint_uart[joint_id-1], Start1, 11, 10);
	HAL_Delay(100);
		HAL_UART_Transmit(joint_uart[joint_id-1], Start2, 11, 10);
	
	return JOINT_SUCCESS;
}
/**
 * @brief 将浮点角度转换为有符号16位整数
 * @param angle 浮点角度值（单位：度）
 * @return 转换后的int16_t值
 * @note 角度范围：-360.0 ~ +360.0度
 *       精度：0.01度
 */
int16_t float_to_int16(float angle)
{
	// 精度定义：每个单位代表0.01度
#define ANGLE_SCALE    100.0f    // 保留2位小数精度
#define MAX_ANGLE      360.0f    // 最大角度
#define MIN_ANGLE     -360.0f    // 最小角度
    // 1. 限制角度范围
    if (angle > MAX_ANGLE) {
        angle = MAX_ANGLE;
    } else if (angle < MIN_ANGLE) {
        angle = MIN_ANGLE;
    }
    
    // 2. 转换为定点数（放大100倍）
    float scaled_angle = angle * ANGLE_SCALE;
    
    // 3. 四舍五入取整
    int32_t temp;
    if (scaled_angle >= 0) {
        temp = (int32_t)(scaled_angle + 0.5f);
    } else {
        temp = (int32_t)(scaled_angle - 0.5f);
    }
    
    // 4. 检查是否超出int16范围
    if (temp > 32767) {
        return 32767;  // int16最大值
    } else if (temp < -32768) {
        return -32768; // int16最小值
    }
    
    return (int16_t)temp;
}

void joint_move(int joint_id,int16_t value){
		HAL_UART_Transmit(joint_uart[joint_id-1], plus_mode1, 10, 100);
		int16_t temp=value;
		if(value!=0)
		{
			//判断设定速度正负确定Telegram[9]与Telegram[10]
			if(value>0)
			{
				Telegram1[10]=0x00;
				Telegram1[9]=0x00;
				printf("+");
				
			}
			else
			{
				Telegram1[10]=0xFF;
				Telegram1[9]=0xFF;
				printf("-");
			}
		}	
		
		//解析Setangle，低8位填入Telegram[7]，高8位填入Telegram[8]
		Telegram1[7] = temp&0XFF;//换算出期望角度低8位//
		Telegram1[8] = temp>>8;//换算出期望角度高8位//
		//根据Telegram[1]~Telegram[10]计算CRC8校验和//
		Telegram1[11] = CalcCRC(Telegram1,10);//计算校验位//
		HAL_UART_Transmit(joint_uart[joint_id-1], Telegram1, 13, 100);
//		HAL_UART_Transmit(&huart1, Telegram1, 13, 100);
		HAL_UART_Transmit(joint_uart[joint_id-1], Controlword1, 11, 100);
		HAL_UART_Transmit(joint_uart[joint_id-1], Controlword2, 11, 100);
		Telegram1[10]=0x00;
		Telegram1[9]=0x00;
		Telegram1[7] =0x00;
		Telegram1[8] =0x00;
		Telegram1[11]=0x00;
}
void joint_target(int joint_id,int16_t value){
		HAL_UART_Transmit(joint_uart[joint_id-1], target_mode, 10, 100);
		int16_t temp=value;
		if(value!=0)
		{
			//判断设定速度正负确定Telegram[9]与Telegram[10]
			if(value>0)
			{
				Telegram1[10]=0x00;
				Telegram1[9]=0x00;
			}
			else
			{
				Telegram1[10]=0xFF;
				Telegram1[9]=0xFF;
			}
		}	
		
		//解析Setangle，低8位填入Telegram[7]，高8位填入Telegram[8]
		Telegram1[7] = temp&0XFF;//换算出期望角度低8位//
		Telegram1[8] = temp>>8;//换算出期望角度高8位//
		//根据Telegram[1]~Telegram[10]计算CRC8校验和//
		Telegram1[11] = CalcCRC(Telegram1,10);//计算校验位//
		HAL_UART_Transmit(joint_uart[joint_id-1], Telegram1, 13, 100);
//		HAL_UART_Transmit(&huart1, Telegram1, 13, 100);
		HAL_UART_Transmit(joint_uart[joint_id-1], Controlword1, 11, 100);
		HAL_UART_Transmit(joint_uart[joint_id-1], Controlword4, 11, 100);
		Telegram1[10]=0x00;
		Telegram1[9]=0x00;
		Telegram1[7] =0x00;
		Telegram1[8] =0x00;
		Telegram1[11]=0x00;
}
void joint_homing(int joint_id){
	if(joint_id!=2){
	HAL_UART_Transmit(joint_uart[joint_id-1], plus_mode1, 10, 100);
	HAL_UART_Transmit(joint_uart[joint_id-1], homing_mode, 10, 100);
	HAL_UART_Transmit(joint_uart[joint_id-1], homing_method, 10, 100);
	HAL_UART_Transmit(joint_uart[joint_id-1], Controlword1, 11, 100);
	HAL_UART_Transmit(joint_uart[joint_id-1], Controlword3, 11, 100);
	}
	if(joint_id==2){
		
			HAL_UART_Transmit(joint_uart[joint_id-1], plus_mode1, 10, 100);
	HAL_UART_Transmit(joint_uart[joint_id-1], homing_mode, 10, 100);
	HAL_UART_Transmit(joint_uart[joint_id-1], homing_method1, 10, 100);
	HAL_UART_Transmit(joint_uart[joint_id-1], Controlword1, 11, 100);
	HAL_UART_Transmit(joint_uart[joint_id-1], Controlword3, 11, 100);
	}
}
void joint_position_search(int joint_id){
	HAL_UART_Transmit(joint_uart[joint_id-1], Position_actual_value, 9, 100);

//	HAL_UART_Transmit(joint_uart[joint_id-1], Position_actual_value1, 1, 100);
	HAL_UART_Receive_IT(joint_uart[joint_id-1],joint1_RxPacket,13);
}
