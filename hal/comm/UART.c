/*
 * UART.c
 *
 *  Created on: 07.04.2020
 *      Author: OK / ED
 */

#include "UART.h"

#if defined(USE_UART_INTERFACE)

#define UART_INTR_PRI		5
#define UART_BUFFER_SIZE 	32
#define UART_TIMEOUT_VALUE 	10   // timeout value in 0.5ms (e.g. 10 => 5ms)

static volatile char UARTRxBuffer[UART_BUFFER_SIZE];
static volatile char UARTTxBuffer[UART_BUFFER_SIZE];

static volatile int UARTTxWritePtr;
static volatile int UARTTxReadPtr;

static volatile int UARTRxWritePtr;
static volatile int UARTRxReadPtr;

volatile uint8_t UARTTimeoutFlag;
volatile uint32_t UARTTimeoutTimer;

USART_TypeDef* actualUart = USART3;

#if defined(USE_USART1_ON_PB6_PB7)
	void __attribute__ ((interrupt))USART1_IRQHandler(void);
#elif defined(USE_USART3_ON_PD8_PD9)
	void __attribute__ ((interrupt))USART3_IRQHandler(void);
#endif

/* initialize UART3/UART6 (bitrate code: 0..11) */
void uart_init(uint32_t baudRate)
{
#if defined(USE_USART1_ON_PB6_PB7)

	actualUart = USART1;

	// activate UART1 and GPIOB
	USART_DeInit(USART1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	// assign UART1-Pins (PB6 and PB7)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

#elif defined(USE_USART3_ON_PD8_PD9)

	actualUart = USART3;

	// activate UART3
	USART_DeInit(USART3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// activate GPIOD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// assign UART3 pins (PD8 und PD9)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

#else
	#error "UART pins not defined!"
#endif

	// configure UART
	USART_InitTypeDef UART_InitStructure;
	USART_StructInit(&UART_InitStructure);
	switch(baudRate)
	{
    	case 0:
    		UART_InitStructure.USART_BaudRate = 9600;
    		break;
    	case 1:
    		UART_InitStructure.USART_BaudRate = 14400;
    		break;
    	case 2:
    		UART_InitStructure.USART_BaudRate = 19200;
    		break;
    	case 3:
    		UART_InitStructure.USART_BaudRate = 28800;
    		break;
    	case 4:
    		UART_InitStructure.USART_BaudRate = 38400;
    		break;
    	case 5:
    		UART_InitStructure.USART_BaudRate = 57600;
    		break;
    	case 6:
    		UART_InitStructure.USART_BaudRate = 76800;
    		break;
    	case 7:
    		UART_InitStructure.USART_BaudRate = 115200;
    		break;
    	case 8:
    		UART_InitStructure.USART_BaudRate = 230400;
    		break;
    	case 9:
    		UART_InitStructure.USART_BaudRate = 250000;
    		break;
    	case 10:
    		UART_InitStructure.USART_BaudRate = 500000;
    		break;
    	case 11:
    		UART_InitStructure.USART_BaudRate = 1000000;
    		break;
    	default:
    		UART_InitStructure.USART_BaudRate = 9600;
    		break;
	}

#if (BOARD_CPU == STM32F103)
	UART_InitStructure.USART_BaudRate >>= 1;
#elif  (BOARD_CPU == STM32F205)
	// do not adapt baudrate
#else
	#error "baudrate not defined for selected BOARD_CPU!"
#endif

	USART_Init(actualUart, &UART_InitStructure);

	// activate interrupt for UART
	NVIC_InitTypeDef NVIC_InitStructure;
#if defined(USE_USART1_ON_PB6_PB7)
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel; // STM32F103
#elif defined(USE_USART3_ON_PD8_PD9)
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
#endif
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART_INTR_PRI;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearFlag(actualUart, USART_FLAG_CTS | USART_FLAG_LBD  | USART_FLAG_TXE  |
			USART_FLAG_TC  | USART_FLAG_RXNE | USART_FLAG_IDLE |
			USART_FLAG_ORE | USART_FLAG_NE   | USART_FLAG_FE | USART_FLAG_PE);

	USART_ITConfig(actualUart, USART_IT_PE  ,DISABLE);
	USART_ITConfig(actualUart, USART_IT_TXE ,ENABLE);
	USART_ITConfig(actualUart, USART_IT_TC  ,ENABLE);
	USART_ITConfig(actualUart, USART_IT_RXNE,ENABLE);
	USART_ITConfig(actualUart, USART_IT_IDLE,DISABLE);
	USART_ITConfig(actualUart, USART_IT_LBD ,DISABLE);
	USART_ITConfig(actualUart, USART_IT_CTS ,DISABLE);
	USART_ITConfig(actualUart, USART_IT_ERR ,DISABLE);

	USART_Cmd(actualUart, ENABLE);
}

#if defined(USE_USART1_ON_PB6_PB7)
void USART1_IRQHandler(void)
#elif defined(USE_USART3_ON_PD8_PD9)
void USART3_IRQHandler(void)
#endif
{
	int i;

	// Ist ein Zeichen  angekommen?
	if(actualUart->SR & USART_FLAG_RXNE)
	{
		//Wenn RS485 gerade auf Senden geschaltet ist, dann ist
		//es ein Echo, das hier ignoriert wird.
		if(tmcm_isUartSending())
		{
			i=actualUart->DR;
		}
		else
		{
			//Zeichen in den Empfangspuffer kopieren
			i = UARTRxWritePtr+1;
			if(i==UART_BUFFER_SIZE) i=0;

			if(i!=UARTRxReadPtr)
			{
				UARTRxBuffer[UARTRxWritePtr]=actualUart->DR;
				UARTRxWritePtr=i;
			}

			//Empfangs-Timeout auf Startwert setzen
			UARTTimeoutTimer = UART_TIMEOUT_VALUE;
		}
	}

	//Kann das nächste Zeichen gesendet werden?
	if(actualUart->SR & USART_FLAG_TXE)
	{		if(UARTTxWritePtr!=UARTTxReadPtr)
		{
			tmcm_setUartToSendMode();
			actualUart->DR = UARTTxBuffer[UARTTxReadPtr++];
			if(UARTTxReadPtr == UART_BUFFER_SIZE)
				UARTTxReadPtr=0;
		}
		else
		{
			//Sendeinterrupt deaktivieren, wenn kein Zeichen im Sendepuffer ist
			USART_ITConfig(actualUart, USART_IT_TXE ,DISABLE);
		}
	}

	//Allerletztes Bit gesendet?
	if(actualUart->SR & USART_FLAG_TC)
	{
		USART_ClearITPendingBit(actualUart, USART_IT_TC);
		if(UARTTxReadPtr == UARTTxWritePtr)
		{
			tmcm_setUartToReceiveMode();
		}
	}
}

void uart_write(char ch)
{
	//Zeichen in die Warteschlange stellen
	int i = UARTTxWritePtr+1;
	if(i == UART_BUFFER_SIZE)
		i = 0;

	if(i != UARTTxReadPtr)
	{
		UARTTxBuffer[UARTTxWritePtr] = ch;
		UARTTxWritePtr = i;

		//Sendeinterrupt aktivieren
		USART_ITConfig(actualUart, USART_IT_TXE, ENABLE);
	}
}

char uart_read(char *ch)
{
	// kein Zeichen vorhanden?
	if(UARTRxReadPtr == UARTRxWritePtr) return false;

	// Zeichen aus dem Puffer holen
	*ch = UARTRxBuffer[UARTRxReadPtr++];
	if(UARTRxReadPtr == UART_BUFFER_SIZE)
		UARTRxReadPtr = 0;

	return true;
}

uint32_t uart_checkTimeout()
{
	if(UARTTimeoutFlag)
	{
		UARTTimeoutFlag = false;
		return true;
	}
	return false;
}

#endif
