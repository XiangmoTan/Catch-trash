#include "openmv_com.h"

/************************************************
                   通信模块
************************************************/
u8 USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART2_REC_LEN个字节.
u8 USART_RX_BUF_NOT[USART_REC_LEN_NOT];
u16 USART_RX_STA = 0; // 接收状态标记
u16 x_error = 0, width = 0, hight = 0, distance = 0;
u8 Count = 0;     // 接受OpenMV数据里用到的
u8 Count_NOT = 0; // 接受OpenMV数据里用到的

void Openmv_Receive_Data(int16_t data);
void Openmv_Receive_Data_NOT(int16_t data);
// void uart_init(u32 bound)
// {
//     // GPIO端口设置
//     GPIO_InitTypeDef GPIO_InitStructure;
//     USART_InitTypeDef USART_InitStructure;
//     NVIC_InitTypeDef NVIC_InitStructure;

//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); // 使能USART1，GPIOA时钟

//     // USART1_TX   GPIOA.9
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PA.9
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
//     GPIO_Init(GPIOA, &GPIO_InitStructure);          // 初始化GPIOA.9

//     // USART1_RX	  GPIOA.10初始化
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;            // PA10
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
//     GPIO_Init(GPIOA, &GPIO_InitStructure);                // 初始化GPIOA.10

//     // Usart1 NVIC 配置
//     NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级1
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        // 子优先级1
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道使能
//     NVIC_Init(&NVIC_InitStructure);                           // 根据指定的参数初始化VIC寄存器

//     // USART 初始化设置

//     USART_InitStructure.USART_BaudRate = bound;                                     // 串口波特率
//     USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 字长为8位数据格式
//     USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 一个停止位
//     USART_InitStructure.USART_Parity = USART_Parity_No;                             // 无奇偶校验位
//     USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
//     USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // 收发模式

//     USART_Init(USART1, &USART_InitStructure);      // 初始化串口1
//     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启串口接受中断
//     USART_Cmd(USART1, ENABLE);                     // 使能串口1
// }

// void USART1_IRQHandler(void) // 串口1中断服务程序
// {
//     u8 Res = 0;
//     if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // 接收中断
//     {

//         //		New_Motor1Pluse = Read_Encoder(2);
//         //		New_Motor2Pluse = Read_Encoder(4);
//         //
//         //		Sigma_Motor1Pluse+=New_Motor1Pluse;
//         //		Sigma_Motor2Pluse+=New_Motor2Pluse;
//         //
//         //		Distance_Motor1Curret=(Sigma_Motor1Pluse*2*3.14*WHEEL_R)/(RESOLUTION_TOTAL*REDUCTION_RATIO);
//         //		Distance_Motor2Curret=(Sigma_Motor2Pluse*2*3.14*WHEEL_R)/(RESOLUTION_TOTAL*REDUCTION_RATIO);

//         USART_ClearITPendingBit(USART1, USART_IT_RXNE); // 清除中断标志
//         Res = USART_ReceiveData(USART1);                // 串口2接收数据
//         Openmv_Receive_Data_NOT(Res);                   // openmv数据处理函数
//         Openmv_Receive_Data(Res);                       // openmv数据处理函数
//     }
// }

void Openmv_Receive_Data(int16_t data)
{
    static u8 state = 0;
    if ((state == 0) && (data == 0x2C))
    {
        state = 1;
        USART_RX_BUF[Count++] = data;
    }
    else if ((state == 1) && (data == 0x12))
    {
        state = 2;
        USART_RX_BUF[Count++] = data;
    }
    else if (state == 2)
    {
        USART_RX_BUF[Count++] = data;
        if (Count > 10 || data == 0x5B)
            state = 3;
    }
    else if (state == 3) // state == 3  检测是否接受到结束标志
    {
        if (USART_RX_BUF[Count - 1] == 0x5B)
        {
            state = 0;
            USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); // 关闭DTSABLE中断
            x_error = USART_RX_BUF[2];
            width = USART_RX_BUF[3];
            hight = USART_RX_BUF[4];
            distance = USART_RX_BUF[5];
            USART_RX_STA = 1;
            Count = 0;
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启ENABLE中断
        }
        else // 错误
        {
            state = 0;
            Count = 0;
        }
    }
    else // 错误
    {
        state = 0;
        Count = 0;
    }
}

void Openmv_Receive_Data_NOT(int16_t data_NOT)
{
    static u8 state_NOT = 0;
    if ((state_NOT == 0) && (data_NOT == 0x3C))
    {
        state_NOT = 1;
        USART_RX_BUF_NOT[Count_NOT++] = data_NOT;
    }
    else if ((state_NOT == 1) && (data_NOT == 0x22))
    {
        state_NOT = 2;
        USART_RX_BUF_NOT[Count_NOT++] = data_NOT;
    }
    else if (state_NOT == 2)
    {
        USART_RX_BUF_NOT[Count_NOT++] = data_NOT;
        if (Count_NOT > 10 || data_NOT == 0x6B)
            state_NOT = 3;
    }
    else if (state_NOT == 3) // state_NOT == 3  检测是否接受到结束标志
    {
        if (USART_RX_BUF_NOT[Count_NOT - 1] == 0x6B)
        {
            state_NOT = 0;
            USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); // 关闭DTSABLE中断
            distance = USART_RX_BUF_NOT[2];
            USART_RX_STA = 5;
            Count_NOT = 0;
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启ENABLE中断
        }
        else // 错误
        {
            state_NOT = 0;
            Count_NOT = 0;
        }
    }
    else // 错误
    {
        state_NOT = 0;
        Count_NOT = 0;
    }
}
