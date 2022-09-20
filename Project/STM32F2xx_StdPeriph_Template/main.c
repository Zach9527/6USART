/**
  ******************************************************************************
  * @file    Project/STM32F2xx_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Main program body
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
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
extern struct fifo recvfifo[6];
extern struct fifo sendfifo[6];
extern struct pBuf recvcontent[6];
extern USART_TypeDef* COM_USART[COMn];
int readTotal;
uint8 timer1Start = 0;
#ifdef UARTSOFTWARE
struct virtual_UART virUart = {0,0,0,{0},0};
#endif
static void USART_Config(void)
{
    USART_InitTypeDef USART_InitStructure;
    COM_TypeDef i;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    for(i = 0; i < COM_MAX; i++)
    {
        NVIC_Config(i);
        STM_EVAL_COMInit(i, &USART_InitStructure);
    }
}
void usartChipInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_14 |GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_0);
  GPIO_SetBits(GPIOA,GPIO_Pin_11);
  GPIO_SetBits(GPIOB,GPIO_Pin_12);
  GPIO_SetBits(GPIOC,GPIO_Pin_4);
  GPIO_SetBits(GPIOC,GPIO_Pin_8);
  GPIO_SetBits(GPIOC,GPIO_Pin_14);
  GPIO_ResetBits(GPIOA,GPIO_Pin_1);
  GPIO_ResetBits(GPIOA,GPIO_Pin_12);
  GPIO_ResetBits(GPIOB,GPIO_Pin_13);
  GPIO_ResetBits(GPIOC,GPIO_Pin_5);
  GPIO_ResetBits(GPIOC,GPIO_Pin_9);
  GPIO_ResetBits(GPIOC,GPIO_Pin_15);

}
void allLedOff(void)
{
    STM_EVAL_LEDOn(LED1);
    STM_EVAL_LEDOn(LED2);
    STM_EVAL_LEDOn(LED3);
    STM_EVAL_LEDOn(LED4);
}

void sysInit(void)
{
    int fifonum;
    usartChipInit();
    USART_Config();
#ifdef UARTSOFTWARE
    IOConfig();//模拟串口
    TIM2_Int_Init(125,50);//        125*50/60000000 = 104us约等于波特率为9600传输1bit的时间定时器的频率为60M
#endif
    STM_EVAL_LEDInit(LED1);
    STM_EVAL_LEDInit(LED2);	
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);
    allLedOff();
    for(fifonum = 0; fifonum < 6; fifonum++)
    {
        recvFifoOpen(&(recvfifo[fifonum]),fifonum);
    }
    for(fifonum = 0; fifonum < 6; fifonum++)
    {
        sendFifoOpen(&(sendfifo[fifonum]),fifonum);
    }
}

void manyToOne(struct pBuf* pbuf)
{
    int i;
    uint8* SecondHead;
    char terminal = 0, noResult = 0;
    uint8 *buf = pbuf->Ram;
    while(buf != NULL && pbuf->len)
    {
        if (!memcmp(buf, "ZT", 2) && pbuf->len > 2)
        {
            again:
            SecondHead = (uint8*)strstr((char*)(buf + 2), "ZT");
            for (i = 2; i < pbuf->len; i++)
            {
                if (buf[i] == 0xFE&& buf[i-1] == 0x5a)
                {
                    //currentFrameLen = i + 1;
                    if (check_sum(buf,i) == buf[i-2])
                    {
                        //写入
                        fifoWrite(buf,i+1,&sendfifo[1]);
                        pbuf->len = pbuf->len - i - 1;
                        buf = buf + i + 1;
                        noResult = 0;
                    }
                    else if(i == (pbuf->len - 1))
                    {
                        if (SecondHead == NULL)
                        {
                            terminal = 1;
                            break;
                        }
                        else
                        {
                            pbuf->len = pbuf->len - (SecondHead - buf);
                            buf = SecondHead;
                            noResult = 1;
                            goto again;
                        }
                    }
                }
                else if(i == pbuf->len - 1)
                {
                    if (SecondHead == NULL)
                    {
                        terminal = 1;
                        break;
                    }
                    else
                    {
                        pbuf->len = pbuf->len - (SecondHead - buf);
                        buf = SecondHead;
                        noResult = 1;
                        goto again;
                    }
                }
            }
            
        }
        else if(pbuf->len > 1 && memcmp(buf, "ZT", 2))
        {
            buf++;
            pbuf->len--;
        }
        else
        {
            break;
        }
        if (terminal)
        {
            break;
        }
    }
    if (!noResult)
    {
        memcpy(pbuf->Ram, buf, pbuf->len);
        memset(pbuf->Ram + pbuf->len, 0x0, RAMSIZE - pbuf->len);
    }
    else
    {
        pbuf->len = readTotal;
    }

}
void port_Choice(uint8* buf,int writeLen,uint8 port)
{
    switch(port)
    {
        case 0x03:
            fifoWrite(buf,writeLen,&sendfifo[2]);//串口3
            break;
        case 0x04:
            fifoWrite(buf,writeLen,&sendfifo[0]);//串口1
            break;
        case 0x05:
            fifoWrite(buf,writeLen,&sendfifo[5]);//串口6
            break;
        case 0x06:
            fifoWrite(buf,writeLen,&sendfifo[4]);//串口5
            break;
        case 0x07:
            fifoWrite(buf,writeLen,&sendfifo[3]);//串口4
            break;
        default:
            //nothing to do
            break;
    }
}

void oneToMany(struct pBuf* pbuf)
{
    int i;
    uint8* SecondHead;
    uint8 terminal = 0, noResult = 0, serial_port = 0;
    uint8 *buf = pbuf->Ram;
    while(buf != NULL && pbuf->len)
    {
        if (!memcmp(buf, "ZT", 2) && pbuf->len > 2)
        {
            again:
            SecondHead = (uint8*)strstr((char*)(buf + 2), "ZT");
            for (i = 2; i < pbuf->len; i++)
            {
                if (buf[i] == 0xFE&& buf[i-1] == 0x5a)
                {
                    if (check_sum(buf,i) == buf[i-2])
                    {
                        //写入
                        serial_port = buf[3];
                        port_Choice(buf,i+1,serial_port);
                        pbuf->len = pbuf->len - i - 1;
                        buf = buf + i + 1;
                        noResult = 0;
                    }
                    else if(i == (pbuf->len - 1))
                    {
                        if (SecondHead == NULL)
                        {
                            terminal = 1;
                            break;
                        }
                        else
                        {
                            pbuf->len = pbuf->len - (SecondHead - buf);
                            buf = SecondHead;
                            noResult = 1;
                            goto again;
                        }
                    }
                }
                else if(i == pbuf->len - 1)
                {
                    if (SecondHead == NULL)
                    {
                        terminal = 1;
                        break;
                    }
                    else
                    {
                        pbuf->len = pbuf->len - (SecondHead - buf);
                        buf = SecondHead;
                        noResult = 1;
                        goto again;
                    }
                }
            }
            
        }
        else if(pbuf->len > 1 && memcmp(buf, "ZT", 2))
        {
            buf++;
            pbuf->len--;
        }
        else
        {
            break;
        }
        if (terminal)
        {
            break;
        }
    }
    if (!noResult)
    {
        memcpy(pbuf->Ram, buf, pbuf->len);
        memset(pbuf->Ram + pbuf->len, 0x0, RAMSIZE - pbuf->len);
    }
    else
    {
        pbuf->len = readTotal;
    }

}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
  
RCC_ClocksTypeDef RCC_Clocks;
int main(void)
{
    int i = 0;
    /*!< At this stage the microcontroller clock setting is already configured, 
        this is done through SystemInit() function which is called from startup
        file (startup_stm32f2xx.s) before to branch to application main.
        To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f2xx.c file
    */  
    
    /* SysTick end of count event each 10ms */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
    /* Infinite loop */
    sysInit();
    virUart.send_max += sprintf(virUart.sendbuff,"version:V1.00\n");
    virUart.send_flag = 1;
    send_remain_byte();
    TIM_Cmd(TIM2, ENABLE);
    while (1)
    {
        if(getFifoLength(&recvfifo[0]))
        {
            readTotal = fifoRead(&recvcontent[0],&recvfifo[0]);
            if(readTotal < 0)
            {
                allLedOff();
                STM_EVAL_LEDOff(LED1);
                STM_EVAL_LEDOff(LED2);
            }
            if(readTotal > 0)//读取数据存入fifo
            {
                manyToOne(&recvcontent[0]);
            }
        }
        if(getFifoLength(&recvfifo[1]))
        {
            readTotal = fifoRead(&recvcontent[1],&recvfifo[1]);
            if(readTotal < 0)
            {
                allLedOff();
                STM_EVAL_LEDOff(LED1);
                STM_EVAL_LEDOff(LED4);
            }
            if(readTotal > 0)//读取数据存入fifo
            {
                oneToMany(&recvcontent[1]);//连接控制组合
            }
        }
        if(getFifoLength(&recvfifo[2]))
        {
            readTotal = fifoRead(&recvcontent[2],&recvfifo[2]);
            if(readTotal < 0)
            {
                allLedOff();
                STM_EVAL_LEDOff(LED1);
                STM_EVAL_LEDOff(LED3);
                STM_EVAL_LEDOff(LED4);
            }
            if(readTotal > 0)//读取数据存入fifo
            {
                manyToOne(&recvcontent[2]);
            }
        }
        if(getFifoLength(&recvfifo[3]))
        {
            readTotal = fifoRead(&recvcontent[3],&recvfifo[3]);
            if(readTotal < 0)
            {
                allLedOff();
                STM_EVAL_LEDOff(LED1);
                STM_EVAL_LEDOff(LED2);
                STM_EVAL_LEDOff(LED3);
                STM_EVAL_LEDOff(LED4);
            }
            if(readTotal > 0)//读取数据存入fifo
            {
                manyToOne(&recvcontent[3]);
            }
        }
        if(getFifoLength(&recvfifo[4]))
        {
            readTotal = fifoRead(&recvcontent[4],&recvfifo[4]);
            if(readTotal < 0)
            {
                allLedOff();
                STM_EVAL_LEDOff(LED1);
                STM_EVAL_LEDOff(LED2);
                STM_EVAL_LEDOff(LED3);
            }
            if(readTotal > 0)//读取数据存入fifo
            {
                manyToOne(&recvcontent[4]);
            }
        }
        if(getFifoLength(&recvfifo[5]))
        {
            readTotal = fifoRead(&recvcontent[5],&recvfifo[5]);
            if(readTotal < 0)
            {
                allLedOff();
                STM_EVAL_LEDOff(LED1);
                STM_EVAL_LEDOff(LED2);
                STM_EVAL_LEDOff(LED4);
            }
            if(readTotal > 0)//读取数据存入fifo
            {
                manyToOne(&recvcontent[5]);
            }
        }
        for(i = 0; i < 6; i++)
        {
            if(getFifoLength(&sendfifo[i]))
            {
                {
                    USART_ITConfig(COM_USART[i], USART_IT_TXE, ENABLE);
                }
            }
        }
#ifdef UARTSOFTWARE
        if(timer1Start)
        {
            //format soft Uart send buffer and then enable timer 1 
            for(i = 0; i < 6; i++)
            {
                if(recvfifo[i].errorCnt > 0)
                {
                    virUart.send_max += sprintf(virUart.sendbuff,"recv[%d]fifo errorCnt:%d\n",i,recvfifo[i].errorCnt);
                }
            }
            for(i = 0; i < 6; i++)
            {
                if(sendfifo[i].errorCnt > 0)
                {
                    virUart.send_max += sprintf(virUart.sendbuff,"send[%d]fifo errorCnt:%d\n",i,sendfifo[i].errorCnt);
                }
            }
            for(i = 0; i < 6; i++)
            {
                if(recvcontent[i].errorCnt > 0)
                {
                    virUart.send_max += sprintf(virUart.sendbuff,"recv[%d]ram errorCnt:%d\n",i,recvcontent[i].errorCnt);
                }
            }
            timer1Start = 0;
            virUart.send_flag = 1;
            send_remain_byte();
            TIM_Cmd(TIM2, ENABLE);
        }

#endif
    }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
