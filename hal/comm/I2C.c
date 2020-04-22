/****************************************************
  Projekt:

  Modul:   I2C.c
           Funktionen für I2C-Kommunikation
           (Code aus Application Note von ST)

  Datum:   31.8.2011 OK
*****************************************************/
#include "I2C.h"
//#include "stm32f10x_lib.h"
//#include "TMCM-STX.h"
//#include "IO-STX.h"

#if defined TMCM_USE_IIC_INTERFACE
/* I2C SPE mask */
#define CR1_PE_Set              ((u16)0x0001)
#define CR1_PE_Reset            ((u16)0xFFFE)

/* I2C START mask */
#define CR1_START_Set           ((u16)0x0100)
#define CR1_START_Reset         ((u16)0xFEFF)

#define CR1_POS_Set				((u16)0x0800)
#define CR1_POS_Reset      	  	((u16)0xF7FF)

/* I2C STOP mask */
#define CR1_STOP_Set            ((u16)0x0200)
#define CR1_STOP_Reset          ((u16)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((u16)0x0400)
#define CR1_ACK_Reset           ((u16)0xFBFF)

/* I2C ENARP mask */
#define CR1_ENARP_Set           ((u16)0x0010)
#define CR1_ENARP_Reset         ((u16)0xFFEF)

/* I2C NOSTRETCH mask */
#define CR1_NOSTRETCH_Set       ((u16)0x0080)
#define CR1_NOSTRETCH_Reset     ((u16)0xFF7F)

/* I2C registers Masks */
#define CR1_CLEAR_Mask          ((u16)0xFBF5)

/* I2C DMAEN mask */
#define CR2_DMAEN_Set           ((u16)0x0800)
#define CR2_DMAEN_Reset         ((u16)0xF7FF)

/* I2C LAST mask */
#define CR2_LAST_Set            ((u16)0x1000)
#define CR2_LAST_Reset          ((u16)0xEFFF)

/* I2C FREQ mask */
#define CR2_FREQ_Reset          ((u16)0xFFC0)

/* I2C ADD0 mask */
#define OAR1_ADD0_Set           ((u16)0x0001)
#define OAR1_ADD0_Reset         ((u16)0xFFFE)

/* I2C ENDUAL mask */
#define OAR2_ENDUAL_Set         ((u16)0x0001)
#define OAR2_ENDUAL_Reset       ((u16)0xFFFE)

/* I2C ADD2 mask */
#define OAR2_ADD2_Reset         ((u16)0xFF01)

/* I2C F/S mask */
#define CCR_FS_Set              ((u16)0x8000)

/* I2C CCR mask */
#define CCR_CCR_Set             ((u16)0x0FFF)

/* I2C FLAG mask */
#define FLAG_Mask               ((u32)0x00FFFFFF)

/* I2C Interrupt Enable mask */
#define ITEN_Mask               ((u32)0x07000000)


#define I2C_IT_BUF                      ((u16)0x0400)
#define I2C_IT_EVT                      ((u16)0x0200)
#define I2C_IT_ERR                      ((u16)0x0100)


#define I2C_DIRECTION_TX 0
#define I2C_DIRECTION_RX 1

#define OwnAddress1 0x28
#define OwnAddress2 0x30

#define I2C1_DMA_CHANNEL_TX           DMA1_Channel6
#define I2C1_DMA_CHANNEL_RX           DMA1_Channel7

#define I2C2_DMA_CHANNEL_TX           DMA1_Channel4
#define I2C2_DMA_CHANNEL_RX           DMA1_Channel5

#define I2C1_DR_Address              0x40005410
#define I2C2_DR_Address              0x40005810

DMA_InitTypeDef  I2CDMA_InitStructure;
volatile u8 Address;

//Prototypen für Interruptfunktionen
void  __attribute__ ((interrupt)) I2C1_ER_IRQHandler(void);
void  __attribute__ ((interrupt)) I2C2_ER_IRQHandler(void);


/*******************************************************************
   Funktion: I2C_LowLevel_Init()
   Parameter: I2Cx: zu initialisierendes Interface (I2C1 oder I2C2).

   Rückgabewert: ---

   Zweck: Initialisierung des IIC-Interface. Es wird dabei die
          Registersatz des gewünschten IIC-Interface übergeben
          (I2C1 oder I2C2, in der Library definiert).
********************************************************************/
static void I2C_LowLevel_Init(I2C_TypeDef* I2Cx)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  I2C_InitTypeDef   I2C_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Enable the DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  if (I2Cx == I2C1)
  {
    /* I2C1 clock enable */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);  //legt I2C1 auf PB8/PB9 statt PB6/PB7
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable I2C1 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* Release I2C1 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
  else /* I2Cx = I2C2 */
  {
    /* I2C2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    /* I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable I2C2 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
    /* Release I2C2 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }

  /* I2C1 and I2C2 configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = OwnAddress1;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

  #if defined DIFF_PRESSURE_SENSOR_SM9333
  I2C_InitStructure.I2C_ClockSpeed = 400000;  //400kHz (other devices)
  #endif

  I2C_Init(I2C1, &I2C_InitStructure);
  I2C_InitStructure.I2C_OwnAddress1 = OwnAddress2;
  I2C_Init(I2C2, &I2C_InitStructure);

  if (I2Cx == I2C1)
  {
    /* I2C1 TX DMA Channel configuration */
    DMA_DeInit(I2C1_DMA_CHANNEL_TX);
    I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;   /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_BufferSize = 0xFFFF;            /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(I2C1_DMA_CHANNEL_TX, &I2CDMA_InitStructure);

    /* I2C1 RX DMA Channel configuration */
    DMA_DeInit(I2C1_DMA_CHANNEL_RX);
    DMA_Init(I2C1_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
  }
  else /* I2Cx = I2C2 */
  {
    /* I2C2 TX DMA Channel configuration */
    DMA_DeInit(I2C2_DMA_CHANNEL_TX);
    I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C2_DR_Address;
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;   /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_BufferSize = 0xFFFF;            /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(I2C2_DMA_CHANNEL_TX, &I2CDMA_InitStructure);

    /* I2C2 RX DMA Channel configuration */
    DMA_DeInit(I2C2_DMA_CHANNEL_RX);
    DMA_Init(I2C2_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
  }
}


/*******************************************************************
   Funktion: I2C_DMAConfig
   Parameter: I2Cx: verwendetes Interface (I2C1 oder I2C2).
              pBuffer: Zeiger auf Datenpuffer
              BufferSize: Größe des Puffers
              Direction: I2C_DIRECTION_TX oder I2C_DIRECTION_RX

   Rückgabewert: ---

   Zweck: Konfigurieren des DMA-Kanals für den nächsten IIC-Transfer.
          Wird von den Funktionen I2C_Master_BufferRead() und
          I2C_Master_BufferWrite() verwendet.
********************************************************************/
static void I2C_DMAConfig(I2C_TypeDef* I2Cx, u8* pBuffer, u32 BufferSize, u32 Direction)
{
  /* Initialize the DMA with the new parameters */
  if(Direction == I2C_DIRECTION_TX)
  {
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (u32)pBuffer;
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    I2CDMA_InitStructure.DMA_BufferSize = (u32)BufferSize;

    if(I2Cx == I2C1)
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
      DMA_Cmd(I2C1_DMA_CHANNEL_TX, DISABLE);
      DMA_Init(I2C1_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C1_DMA_CHANNEL_TX, ENABLE);
    }
    else
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C2_DR_Address;
      DMA_Cmd(I2C2_DMA_CHANNEL_TX, DISABLE);
      DMA_Init(I2C2_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C2_DMA_CHANNEL_TX, ENABLE);
    }
  }
  else /* Reception */
  {
    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (u32)pBuffer;
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    I2CDMA_InitStructure.DMA_BufferSize = (u32)BufferSize;

    if(I2Cx == I2C1)
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
      DMA_Cmd(I2C1_DMA_CHANNEL_RX, DISABLE);
      DMA_Init(I2C1_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C1_DMA_CHANNEL_RX, ENABLE);
    }
    else
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2C2_DR_Address;
      DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE);
      DMA_Init(I2C2_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C2_DMA_CHANNEL_RX, ENABLE);
    }
  }
}


/*******************************************************************
   Funktion: I2C_Master_BufferRead()
   Parameter: I2Cx: zu verwendendes Interface (I2C1 oder I2C2).
              pBuffer: Zeiger auf Datenpuffer
              NumByteToRead: Anzahl der zu lesenden Bytes
              SlaveAddress: IIC-Adresse des Gerätes

   Rückgabewert: TRUE bei Erfolg
                 FALSE bei Fehler

   Zweck: Lesen von Daten über IIC. Der Datentransfer erfolgt über DMA,
          so daß es auch dann nicht zu Fehlern kommen kann, wenn
          während des Transfers andere Interrupts auftreten.
          Es müssen jedoch mindestens zwei Bytes übertragen werden, bei
          nur einem Byte gibt es Probleme.
          Die Funktion benötigt den IIC-Error-Interrupt, nicht jedoch
          den IIC-Event-Interrupt.
********************************************************************/
uint8_t I2C_Master_BufferRead(I2C_TypeDef* I2Cx, u8* pBuffer,  u32 NumByteToRead, u8 SlaveAddress)
{
  volatile u32 temp = 0;
  volatile u32 Timeout = 0;

  /* Enable I2C errors interrupts (used in all modes: Polling, DMA and Interrupts */
  I2Cx->CR2 |= I2C_IT_ERR;

  /* Configure I2Cx DMA channel */
  I2C_DMAConfig(I2Cx, pBuffer, NumByteToRead, I2C_DIRECTION_RX);
  /* Set Last bit to have a NACK on the last received byte */
  I2Cx->CR2 |= CR2_LAST_Set;
  /* Enable I2C DMA requests */
  I2Cx->CR2 |= CR2_DMAEN_Set;
  Timeout = 0xFFFF;
  /* Send START condition */
  I2Cx->CR1 |= CR1_START_Set;
  /* Wait until SB flag is set: EV5  */
  while ((I2Cx->SR1&0x0001) != 0x0001)
  {
    if(Timeout-- == 0) return false;
  }
  Timeout = 0xFFFF;
  /* Send slave address */
  /* Set the address bit0 for read */
  SlaveAddress |= OAR1_ADD0_Set;
  Address = SlaveAddress;
  /* Send the slave address */
  I2Cx->DR = Address;
  /* Wait until ADDR is set: EV6 */
  while ((I2Cx->SR1&0x0002) != 0x0002)
  {
    if (Timeout-- == 0) return false;
  }
  /* Clear ADDR flag by reading SR2 register */
  temp = I2Cx->SR2;
  if (I2Cx == I2C1)
  {
    /* Wait until DMA end of transfer */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC7));
    /* Disable DMA Channel */
    DMA_Cmd(I2C1_DMA_CHANNEL_RX, DISABLE);
    /* Clear the DMA Transfer Complete flag */
    DMA_ClearFlag(DMA1_FLAG_TC7);
  }
  else /* I2Cx = I2C2*/
  {
    /* Wait until DMA end of transfer */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC5));
    /* Disable DMA Channel */
    DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE);
    /* Clear the DMA Transfer Complete flag */
    DMA_ClearFlag(DMA1_FLAG_TC5);
  }
  /* Program the STOP */
  I2Cx->CR1 |= CR1_STOP_Set;
  /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  while ((I2Cx->CR1&0x200) == 0x200);

  return true;
}


/*******************************************************************
   Funktion: I2C_Master_BufferWrite()
   Parameter: I2Cx: zu verwendendes Interface (I2C1 oder I2C2).
              pBuffer: Zeiger auf Datenpuffer
              NumByteToWrite: Anzahl der zu lesenden Bytes
              SlaveAddress: IIC-Adresse des Gerätes

   Rückgabewert: TRUE bei Erfolg
                 FALSE bei Fehler

   Zweck: Schreiben von Daten über IIC. Der Datentransfer erfolgt über DMA,
          so daß es auch dann nicht zu Fehlern kommen kann, wenn
          während des Transfers andere Interrupts auftreten.
          Die Funktion benötigt den IIC-Error-Interrupt, nicht jedoch
          den IIC-Event-Interrupt.
********************************************************************/
uint8_t I2C_Master_BufferWrite(I2C_TypeDef* I2Cx, u8* pBuffer,  u32 NumByteToWrite, u8 SlaveAddress )
{

  volatile u32 temp = 0;
  volatile u32 Timeout = 0;

  /* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
  I2Cx->CR2 |= I2C_IT_ERR;

  Timeout = 0xFFFF;
  /* Configure the DMA channel for I2Cx transmission */
  I2C_DMAConfig (I2Cx, pBuffer, NumByteToWrite, I2C_DIRECTION_TX);
  /* Enable the I2Cx DMA requests */
  I2Cx->CR2 |= CR2_DMAEN_Set;
  /* Send START condition */
  I2Cx->CR1 |= CR1_START_Set;
  /* Wait until SB flag is set: EV5 */
  while ((I2Cx->SR1&0x0001) != 0x0001)
  {
    if(Timeout-- == 0) return false;
  }
  Timeout = 0xFFFF;
  /* Send slave address */
  /* Reset the address bit0 for write */
  SlaveAddress &= OAR1_ADD0_Reset;
  Address = SlaveAddress;
  /* Send the slave address */
  I2Cx->DR = Address;
  /* Wait until ADDR is set: EV6 */
  while ((I2Cx->SR1&0x0002) != 0x0002)
  {
    if(Timeout-- == 0) return false;
  }

  /* Clear ADDR flag by reading SR2 register */
  temp = I2Cx->SR2;
  if (I2Cx == I2C1)
  {
    /* Wait until DMA end of transfer */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC6));
    /* Disable the DMA1 Channel 6 */
    DMA_Cmd(I2C1_DMA_CHANNEL_TX, DISABLE);
    /* Clear the DMA Transfer complete flag */
    DMA_ClearFlag(DMA1_FLAG_TC6);
  }
  else  /* I2Cx = I2C2 */
  {
    /* Wait until DMA end of transfer */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC4));
    /* Disable the DMA1 Channel 4 */
    DMA_Cmd(I2C2_DMA_CHANNEL_TX, DISABLE);
    /* Clear the DMA Transfer complete flag */
    DMA_ClearFlag(DMA1_FLAG_TC4);
  }

  /* EV8_2: Wait until BTF is set before programming the STOP */
  while ((I2Cx->SR1 & 0x00004) != 0x000004);
  /* Program the STOP */
  I2Cx->CR1 |= CR1_STOP_Set;
  /* Make sure that the STOP bit is cleared by Hardware */
  while ((I2Cx->CR1&0x200) == 0x200);

  return true;
}


/*******************************************************************
   Funktion: InitIIC()
   Parameter: ---
   Rückgabewert: ---

   Zweck: Initialisierung des IIC-Interface.
********************************************************************/
void InitIIC(void)
{
  #if defined DIFF_PRESSURE_SENSOR_SM9333
  I2C_LowLevel_Init(I2C1);
  #else
  #error "I2C initialization not defined"
  #endif
}


/*******************************************************************
   Funktion: I2C1_ER_IRQHandler()
   Parameter: ---
   Rückgabewert: ---

   Zweck: Interrupthandler für den Error-Interrupt des IIC-Interface 1.
          Bei auftretenden Fehlern werden die entsprechenden Fehlerbits
          zurückgesetzt, damit das Interface danach wieder funktioniert.
********************************************************************/
void I2C1_ER_IRQHandler(void)
{
  volatile uint32_t SR1Register=0;

  /* Read the I2C1 status register */
  SR1Register = I2C1->SR1;
  /* If AF = 1 */
  if((SR1Register & 0x0400) == 0x0400)
  {
    I2C1->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If ARLO = 1 */
  if((SR1Register & 0x0200) == 0x0200)
  {
    I2C1->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If BERR = 1 */
  if((SR1Register & 0x0100) == 0x0100)
  {
    I2C1->SR1 &= 0xFEFF;
    SR1Register = 0;
  }

  /* If OVR = 1 */

  if((SR1Register & 0x0800) == 0x0800)
  {
    I2C1->SR1 &= 0xF7FF;
    SR1Register = 0;
  }
}


/*******************************************************************
   Funktion: I2C2_ER_IRQHandler()
   Parameter: ---
   Rückgabewert: ---

   Zweck: Interrupthandler für den Error-Interrupt des IIC-Interface 2.
          Bei auftretenden Fehlern werden die entsprechenden Fehlerbits
          zurückgesetzt, damit das Interface danach wieder funktioniert.
********************************************************************/
void I2C2_ER_IRQHandler(void)
{
  volatile uint32_t SR1Register=0;

  /* Read the I2C1 status register */
  SR1Register = I2C2->SR1;
  /* If AF = 1 */
  if((SR1Register & 0x0400) == 0x0400)
  {
    I2C2->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If ARLO = 1 */
  if((SR1Register & 0x0200) == 0x0200)
  {
    I2C2->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If BERR = 1 */
  if((SR1Register & 0x0100) == 0x0100)
  {
    I2C2->SR1 &= 0xFEFF;
    SR1Register = 0;
  }

  /* If OVR = 1 */
  if((SR1Register & 0x0800) == 0x0800)
  {
    I2C2->SR1 &= 0xF7FF;
    SR1Register = 0;
  }
}

#endif
