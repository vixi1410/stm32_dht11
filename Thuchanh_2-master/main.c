#include "stm32f10x.h"                  

void timer2_Init(void);
void Delay_1ms(uint32_t time_ms);
void DHT11_Init(uint8_t *tem, uint8_t *hum);
void UART_Init(void);
void UART_SendChar(char c);
void UART_SendString(char *str);

void UART_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 9600;  
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);
}

void UART_SendChar(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, c);
}

void UART_SendString(char *str)
{
    while (*str)
    {
        UART_SendChar(*str++);
    }
}

void timer2_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef timerInit;
    timerInit.TIM_CounterMode = TIM_CounterMode_Up;
    timerInit.TIM_Period = 0xFFFF;
    timerInit.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInit(TIM2, &timerInit);
    TIM_Cmd(TIM2, ENABLE);
}

void Delay_1ms(uint32_t time_ms)
{
    for (uint32_t i = 0; i < time_ms; i++)
    {
        TIM_SetCounter(TIM2, 0);
        while (TIM_GetCounter(TIM2) < 1000);
    }
}

void DHT11_Init(uint8_t *tem, uint8_t *hum) 
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef gpioInit;
    gpioInit.GPIO_Pin = GPIO_Pin_12;
    gpioInit.GPIO_Mode = GPIO_Mode_Out_OD;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioInit);

    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    Delay_1ms(100);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);

    TIM_SetCounter(TIM2,0);
    while(TIM_GetCounter(TIM2) < 10){
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)){
            break;
        }
    }
    
    uint16_t u16Tim;
    uint8_t u8Buff[5];
    
    u16Tim = TIM_GetCounter(TIM2);
    if(u16Tim >= 10){
        UART_SendString("Error: Start signal timing failed!\n");
        while(1); 
    }
    
    TIM_SetCounter(TIM2,0);
    while(TIM_GetCounter(TIM2) < 45){
        if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)){
            break;
        }
    }
    
    u16Tim = TIM_GetCounter(TIM2);
    if((u16Tim >= 45)||(u16Tim <= 5)){
        UART_SendString("Error: Response timing failed!\n");
        while(1);
    }
    
    TIM_SetCounter(TIM2,0);
    while(TIM_GetCounter(TIM2) < 90){
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)){
            break;
        }
    }
    
    u16Tim = TIM_GetCounter(TIM2);
    if((u16Tim >= 90)||(u16Tim <= 70)){
        UART_SendString("Error: 80us low signal failed!\n");
        while(1);
    }
    
    TIM_SetCounter(TIM2,0);
    while(TIM_GetCounter(TIM2) < 95){
        if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)){
            break;
        }
    }
    
    u16Tim = TIM_GetCounter(TIM2);
    if((u16Tim >= 95)||(u16Tim <= 75)){
        UART_SendString("Error: 80us high signal failed!\n");
        while(1);
    }

    // Ð?c d? li?u t? c?m bi?n DHT11
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 8; ++j) {
            TIM_SetCounter(TIM2, 0);
            while (TIM_GetCounter(TIM2) < 65) {
                if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
                    break;
                }
            }

            u16Tim = TIM_GetCounter(TIM2);
            if ((u16Tim >= 65) || (u16Tim <= 45)) {
                UART_SendString("Error: Bit timing error (65us low)!\n");
                while (1);
            }

            TIM_SetCounter(TIM2, 0);
            while (TIM_GetCounter(TIM2) < 80) {
                if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
                    break;
                }
            }

            u16Tim = TIM_GetCounter(TIM2);
            if ((u16Tim >= 80) || (u16Tim <= 10)) {
                UART_SendString("Error: Bit timing error (80us high)!\n");
                while (1);
            }

            u8Buff[i] <<= 1;
            if (u16Tim > 45) {
                u8Buff[i] |= 1;
            } else {
                u8Buff[i] &= ~1;
            }
        }
    }

    // Ki?m tra checksum
    uint8_t u8CheckSum = u8Buff[0] + u8Buff[1] + u8Buff[2] + u8Buff[3];
    if (u8CheckSum != u8Buff[4]) {
        UART_SendString("Error: Checksum mismatch!\n");
        while(1);
    }

    *tem = u8Buff[0];
    *hum = u8Buff[2];
}

int main() {
    UART_Init();
    timer2_Init();
		char buffer[50];
		uint8_t humidity, temperature;
    while (1) {
				DHT11_Init(&humidity,&temperature);
				sprintf(buffer, "Temperature: %d*C, Humidity: %d%%\n", temperature, humidity);
        UART_SendString(buffer);
        Delay_1ms(500);
    }
}