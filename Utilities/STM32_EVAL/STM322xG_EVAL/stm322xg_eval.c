/**
  ******************************************************************************
  * @file    stm322xg_eval.c
  * @author  MCD Application Team
  * @version V5.0.3
  * @date    09-March-2012
  * @brief   This file provides
  *            - set of firmware functions to manage Leds, push-button and COM ports
  *            - low level initialization functions for SD card (on SDIO) and
  *              serial EEPROM (sEE)
  *          available on STM322xG-EVAL evaluation board(MB786) RevA and RevB 
  *          from STMicroelectronics.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm322xg_eval.h"
#include "main.h"
#define RECV_FIFOSIZE  1024
#define SEND_FIFOSIZE  1024
uint8 recvbuf1[RECV_FIFOSIZE] = {0};
uint8 recvbuf2[RECV_FIFOSIZE] = {0};
uint8 recvbuf3[RECV_FIFOSIZE] = {0};
uint8 recvbuf4[RECV_FIFOSIZE] = {0};
uint8 recvbuf5[RECV_FIFOSIZE] = {0};
uint8 recvbuf6[RECV_FIFOSIZE] = {0};
uint8 *recvFifoBuffer[6] = {recvbuf1,recvbuf2,recvbuf3,recvbuf4,recvbuf5,recvbuf6};
struct fifo recvfifo[6] = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

uint8 sendbuf1[SEND_FIFOSIZE] = {0};
uint8 sendbuf2[SEND_FIFOSIZE] = {0};
uint8 sendbuf3[SEND_FIFOSIZE] = {0};
uint8 sendbuf4[SEND_FIFOSIZE] = {0};
uint8 sendbuf5[SEND_FIFOSIZE] = {0};
uint8 sendbuf6[SEND_FIFOSIZE] = {0};
uint8 *sendFifoBuffer[6] = {sendbuf1,sendbuf2,sendbuf3,sendbuf4,sendbuf5,sendbuf6};
struct fifo sendfifo[6] = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
struct pBuf recvcontent[6] = {{{0},0,0,0,0},{{0},0,0,0,0},{{0},0,0,0,0},{{0},0,0,0,0},{{0},0,0,0,0},{{0},0,0,0,0}};



#ifdef UARTSOFTWARE

extern struct virtual_UART virUart;
extern uint8 recvStat;

#endif
/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup STM322xG_EVAL
  * @{
  */   
    
/** @defgroup STM322xG_EVAL_LOW_LEVEL 
  * @brief This file provides firmware functions to manage Leds, push-buttons, 
  *        COM ports, SD card on SDIO and serial EEPROM (sEE) available on 
  *        STM322xG-EVAL evaluation board from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM322xG_EVAL_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM322xG_EVAL_LOW_LEVEL_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM322xG_EVAL_LOW_LEVEL_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM322xG_EVAL_LOW_LEVEL_Private_Variables
  * @{
  */ 
  //LED
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN,
                                 LED4_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK, LED3_GPIO_CLK,
                                 LED4_GPIO_CLK};
//USART
USART_TypeDef* COM_USART[COMn] = {EVAL_COM1,EVAL_COM2,EVAL_COM3,EVAL_COM4,EVAL_COM5,EVAL_COM6}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT,EVAL_COM2_TX_GPIO_PORT,EVAL_COM3_TX_GPIO_PORT,
                                EVAL_COM4_TX_GPIO_PORT,EVAL_COM5_TX_GPIO_PORT,EVAL_COM6_TX_GPIO_PORT};
 
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT,EVAL_COM2_RX_GPIO_PORT,EVAL_COM3_RX_GPIO_PORT,
                                EVAL_COM4_RX_GPIO_PORT,EVAL_COM5_RX_GPIO_PORT,EVAL_COM6_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK,EVAL_COM2_CLK,EVAL_COM3_CLK,EVAL_COM4_CLK,EVAL_COM5_CLK,
                                EVAL_COM6_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK,EVAL_COM2_TX_GPIO_CLK,EVAL_COM3_TX_GPIO_CLK,
                                    EVAL_COM4_TX_GPIO_CLK,EVAL_COM5_TX_GPIO_CLK,EVAL_COM6_TX_GPIO_CLK};
 
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK,EVAL_COM2_RX_GPIO_CLK,EVAL_COM3_RX_GPIO_CLK,
                                    EVAL_COM4_RX_GPIO_CLK,EVAL_COM5_RX_GPIO_CLK,EVAL_COM6_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN,EVAL_COM2_TX_PIN,EVAL_COM3_TX_PIN,EVAL_COM4_TX_PIN,
                                EVAL_COM5_TX_PIN,EVAL_COM6_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN,EVAL_COM2_RX_PIN,EVAL_COM3_RX_PIN,EVAL_COM4_RX_PIN,
                                EVAL_COM5_RX_PIN,EVAL_COM6_RX_PIN};
 
const uint8_t COM_TX_PIN_SOURCE[COMn] = {EVAL_COM1_TX_SOURCE,EVAL_COM2_TX_SOURCE,EVAL_COM3_TX_SOURCE,
                                        EVAL_COM4_TX_SOURCE,EVAL_COM5_TX_SOURCE,EVAL_COM6_TX_SOURCE};

const uint8_t COM_RX_PIN_SOURCE[COMn] = {EVAL_COM1_RX_SOURCE,EVAL_COM2_RX_SOURCE,EVAL_COM3_RX_SOURCE,
                                        EVAL_COM4_RX_SOURCE,EVAL_COM5_RX_SOURCE,EVAL_COM6_RX_SOURCE};
 
const uint8_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF,EVAL_COM2_TX_AF,EVAL_COM3_TX_AF,EVAL_COM4_TX_AF,
                                EVAL_COM5_TX_AF,EVAL_COM6_TX_AF};
 
const uint8_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF,EVAL_COM2_RX_AF,EVAL_COM3_RX_AF,EVAL_COM4_RX_AF,
                                EVAL_COM5_RX_AF,EVAL_COM6_RX_AF};

const uint8_t COM_IRQ[COMn] = {USART1_IRQn,USART2_IRQn,USART3_IRQn,UART4_IRQn,UART5_IRQn,USART6_IRQn};

DMA_InitTypeDef    sEEDMA_InitStructure; 
NVIC_InitTypeDef   NVIC_InitStructure;

/**
  * @}
  */ 


/** @defgroup STM322xG_EVAL_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup STM322xG_EVAL_LOW_LEVEL_Private_Functions
  * @{
  */ 

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void STM_EVAL_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(GPIO_CLK[Led], ENABLE);


  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4  
  * @retval None
  */
void STM_EVAL_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRL = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4 
  * @retval None
  */
void STM_EVAL_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRH = GPIO_PIN[Led];  
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4  
  * @retval None
  */
void STM_EVAL_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}

/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);
    
    if (COM == COM1 || COM == COM6)
    {
    /* Enable UART clock */
        RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    }
    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);
    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);
    
    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    
    GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);
    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    
    GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
    GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);
    
    /* USART configuration */
    USART_Init(COM_USART[COM], USART_InitStruct);
    
    /* Enable USART */
    USART_Cmd(COM_USART[COM], ENABLE);
    //使能发送中断
    USART_ITConfig(COM_USART[COM], USART_IT_RXNE, ENABLE);
}

void NVIC_Config(COM_TypeDef COM)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = COM_IRQ[COM];
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


uint8 check_sum(uint8* pbuf,int len)
{
    int i = 0;
    uint8 checksum = 0;
    if(pbuf == NULL || len < 2)
    {
        return -1;
    }
    for(i=2;i<len-2;i++)
    {
        checksum += pbuf[i];
    }
    return checksum;
}
/****************************************
*************  receive fifo  ***************
****************************************/

void recvFifoOpen(struct fifo* fifo, int num)
{
    struct fifo* pfifo = fifo;
    pfifo->fifoSize = RECV_FIFOSIZE;
    pfifo->pfifoBuf = recvFifoBuffer[num];
    pfifo->pfifoIn = recvFifoBuffer[num];
    pfifo->pfifoOut = recvFifoBuffer[num];
}
void sendFifoOpen(struct fifo* fifo, int num)
{
    struct fifo* pfifo = fifo;
    pfifo->fifoSize = SEND_FIFOSIZE;
    pfifo->pfifoBuf = sendFifoBuffer[num];
    pfifo->pfifoIn = sendFifoBuffer[num];
    pfifo->pfifoOut = sendFifoBuffer[num];
}

int fifoWrite(uint8* pbuf,int len,struct fifo* fifo)
{
    int i;
    struct fifo* pfifo = fifo;
    if(pbuf == NULL)
    {
        return -1;
    }
    for(i = 0; i<len ;i++)
    {
        pfifo->pfifoIn[0] = pbuf[i];
        if(((pfifo->pfifoIn-pfifo->pfifoBuf+1)%pfifo->fifoSize) != pfifo->pfifoOut - pfifo->pfifoBuf) //防止溢出
        {
            pfifo->pfifoIn++;
            if(pfifo->pfifoIn == pfifo->pfifoBuf+pfifo->fifoSize)
            {
                pfifo->pfifoIn = pfifo->pfifoBuf;
            }
        }
        else
        {
            //LED灯点亮，暂时没定
            allLedOff();
            
            STM_EVAL_LEDOff(LED4);
            pfifo->errorCnt++;//fifo溢出计数处理
            return -1;
        }
    }
    return 0;
}

int fifoRead(struct pBuf* buf, struct fifo* fifo)
{
    struct fifo* pfifo = fifo;
    //int len = 0;
    while (pfifo->pfifoIn != pfifo->pfifoOut)
    {
        if(buf->len > (RAMSIZE -1))
        {
            buf->errorCnt++;
            return -1;
        }
        buf->Ram[buf->len] = pfifo->pfifoOut[0];
        pfifo->pfifoOut++;
        if (pfifo->pfifoOut == pfifo->pfifoBuf + pfifo->fifoSize)
        {
            pfifo->pfifoOut = pfifo->pfifoBuf;
        }
        buf->len++;
    }
    return buf->len;
}

uint8 usartTrans(struct fifo* fifo)
{
    struct fifo* pfifo = fifo;
    uint8 dat = 0;
    if(pfifo->pfifoIn != pfifo->pfifoOut)
    {
        dat = pfifo->pfifoOut[0];
        pfifo->pfifoOut++;
        if (pfifo->pfifoOut == pfifo->pfifoBuf + pfifo->fifoSize)
        {
            pfifo->pfifoOut = pfifo->pfifoBuf;
        }
    }
    return dat;
}


int getFifoLength(struct fifo* fifo)
{
    struct fifo* pfifo = fifo;
    if(pfifo == NULL)
    {
        return 0;
    }
    return (pfifo->pfifoIn - pfifo->pfifoOut + pfifo->fifoSize) % pfifo->fifoSize;
}


#ifdef UARTSOFTWARE
 void TIM2_Int_Init(u16 arr,u16 psc)
 {
     TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
     NVIC_InitTypeDef NVIC_InitStructure;
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//TIM2
     TIM_TimeBaseStructure.TIM_Period = arr - 1; //自动重装载值
     TIM_TimeBaseStructure.TIM_Prescaler =psc - 1; //预分频系数
     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数
     TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
     TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
     NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //定时器优先级配置
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
     NVIC_Init(&NVIC_InitStructure); 
     TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能定时器计数中断
 }
//模拟串口IO配置
 void IOConfig(void)
 {
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    //SoftWare Serial TXD
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_8); 
    
    
    //SoftWare Serial RXD
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	 
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource9);
    EXTI_InitStruct.EXTI_Line = EXTI_Line9;
    EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising_Falling; 
    EXTI_InitStruct.EXTI_LineCmd=ENABLE;
    EXTI_Init(&EXTI_InitStruct);
    
    NVIC_InitStructure.NVIC_IRQChannel= EXTI9_5_IRQn ; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
    
}

 void send_remain_byte(void)
 {
     if(virUart.send_cnt >= virUart.send_max)
     {
         virUart.send_flag=0;    //发送完毕
         virUart.send_cnt = 0;
         TIM_Cmd(TIM2,DISABLE);
     }
     else
     {
         recvStat = COM_START_BIT;
         virUart.TXREG=virUart.sendbuff[virUart.send_cnt++];
     }
 }


#endif

/**
  * @}
  */ 


/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */    

/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
