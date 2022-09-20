/**
  ******************************************************************************
  * @file    Project/STM32F2xx_StdPeriph_Template/stm32f2xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f2xx_it.h"
#include "main.h"
#include "stm322xg_eval.h"

extern struct fifo recvfifo[6];
extern struct fifo sendfifo[6];
extern struct virtual_UART virUart;
extern uint8 timer1Start;

uint8 recvStat = COM_STOP_BIT;
uint32_t revTotalCnt = 0;
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/
void USART1_IRQHandler(void)
{
    if(RESET != USART_GetITStatus(EVAL_COM1, USART_IT_RXNE))
    {
        uint8 ch = (uint8)(USART_ReceiveData(EVAL_COM1));
        /* receive data */
        fifoWrite(&ch,1,&(recvfifo[0]));
    }
    if(USART_GetITStatus(EVAL_COM1, USART_IT_TXE) != RESET)
    {
        /* transmit data */
        if(getFifoLength(&sendfifo[0]))
        {
        USART_SendData(EVAL_COM1,usartTrans(&sendfifo[0]));
        }
        else
        {
            USART_ITConfig(EVAL_COM1, USART_IT_TXE, DISABLE);
        }

    }
}
void USART2_IRQHandler(void)
{
    if(RESET != USART_GetITStatus(EVAL_COM2, USART_IT_RXNE))
    {
        uint8 ch = (uint8)(USART_ReceiveData(EVAL_COM2));
        /* receive data */
        fifoWrite(&ch,1,&(recvfifo[1]));
    }
    if(USART_GetITStatus(EVAL_COM2, USART_IT_TXE) != RESET)
    {
        /* transmit data */
        if(getFifoLength(&sendfifo[1]))
        {
            USART_SendData(EVAL_COM2,usartTrans(&sendfifo[1]));
        }
        else
        {
            USART_ITConfig(EVAL_COM2, USART_IT_TXE, DISABLE);
        }
    }

}
void USART3_IRQHandler(void)
{
    if(RESET != USART_GetITStatus(EVAL_COM3, USART_IT_RXNE))
    {
        uint8 ch = (uint8)(USART_ReceiveData(EVAL_COM3));
        /* receive data */
        fifoWrite(&ch,1,&(recvfifo[2]));
    }
    if(USART_GetITStatus(EVAL_COM3, USART_IT_TXE) != RESET)
    {
        /* transmit data */
        if(getFifoLength(&sendfifo[2]))
        {
            USART_SendData(EVAL_COM3,usartTrans(&sendfifo[2]));
        }
        else
        {
            USART_ITConfig(EVAL_COM3, USART_IT_TXE, DISABLE);
        }
    }

}
void UART4_IRQHandler(void)
{
    if(RESET != USART_GetITStatus(EVAL_COM4, USART_IT_RXNE))
    {
        uint8 ch = (uint8)(USART_ReceiveData(EVAL_COM4));
        /* receive data */
        fifoWrite(&ch,1,&(recvfifo[3]));
    }
    if(USART_GetITStatus(EVAL_COM4, USART_IT_TXE) != RESET)
    {
        /* transmit data */
        if(getFifoLength(&sendfifo[3]))
        {
            USART_SendData(EVAL_COM4,usartTrans(&sendfifo[3]));
        }
        else
        {
            USART_ITConfig(EVAL_COM4, USART_IT_TXE, DISABLE);
        }

    }

}
void UART5_IRQHandler(void)
{
    if(RESET != USART_GetITStatus(EVAL_COM5, USART_IT_RXNE))
    {
        uint8 ch = (uint8)(USART_ReceiveData(EVAL_COM5));
        /* receive data */
        fifoWrite(&ch,1,&(recvfifo[4]));
    }
    if(USART_GetITStatus(EVAL_COM5, USART_IT_TXE) != RESET)
    {
        /* transmit data */
        if(getFifoLength(&sendfifo[4]))
        {
            USART_SendData(EVAL_COM5,usartTrans(&sendfifo[4]));
        }
        else
        {
            USART_ITConfig(EVAL_COM5, USART_IT_TXE, DISABLE);
        }

    }

}
void USART6_IRQHandler(void)
{
    if(RESET != USART_GetITStatus(EVAL_COM6, USART_IT_RXNE))
    {
        uint8 ch = (uint8)(USART_ReceiveData(EVAL_COM6));
        /* receive data */
        fifoWrite(&ch,1,&(recvfifo[5]));
    }
    if(USART_GetITStatus(EVAL_COM6, USART_IT_TXE) != RESET)
    {
        /* transmit data */
        if(getFifoLength(&sendfifo[5]))
        {
            USART_SendData(EVAL_COM6,usartTrans(&sendfifo[5]));
        }
        else
        {
            USART_ITConfig(EVAL_COM6, USART_IT_TXE, DISABLE);
        }

    }

}

#ifdef UARTSOFTWARE
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line9) != RESET)
    {
        if(OI_RXD == 1) 
        {
            if(recvStat == COM_STOP_BIT)
            {
                recvStat = COM_START_BIT;
                timer1Start = 1;
                
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}
//定时器2中断
void TIM2_IRQHandler(void)
{  
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        if(virUart.send_flag)
        {
            if(recvStat == COM_START_BIT)
            {
                TXD_L();
                recvStat++;
                return;
             }
            if(recvStat == COM_STOP_BIT)
            {
                TXD_H();
                send_remain_byte();
                return;
            }
            if(virUart.TXREG & 0x01)
            {
                TXD_H();
                virUart.TXREG = virUart.TXREG >> 1; //数据右移
                recvStat++;
             }
            else
            {
                TXD_L();
                virUart.TXREG= virUart.TXREG >> 1;
                recvStat++;
             }

        }
    }
}

#endif
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
