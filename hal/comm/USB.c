/*
 * USB.c
 *
 *  Created on: 31.03.2020
 *      Author: ED / OK
 */

#include "USB.h"

#if defined(USE_USB_INTERFACE)

void __attribute__ ((interrupt))OTG_FS_IRQHandler(void);

#if	(BOARD_CPU == STM32F205)

typedef struct
{
	uint32_t bitrate;
	uint8_t  format;
	uint8_t  paritytype;
	uint8_t  datatype;
} LINE_CODING;

#define VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT     50
#define VIRTUAL_COM_PORT_SIZ_STRING_SERIAL      26

#define USBD_LANGID_STRING              0x409
#define USBD_MANUFACTURER_STRING        "Trinamic Motion Control"
#define USBD_CONFIGURATION_FS_STRING    "VCP Config"
#define USBD_INTERFACE_FS_STRING        "VCP Interface"

#define USB_BUFFER_SIZE 64

static uint16_t VCP_Init(void);
static uint16_t VCP_DeInit(void);
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len);

USB_OTG_CORE_HANDLE USB_OTG_dev;				// handle for USB-Core functions
static uint8_t USBTxBuffer[USB_BUFFER_SIZE];
static uint32_t USBTxCounter;
extern uint8_t  APP_Rx_Buffer[];
extern uint32_t APP_Rx_ptr_in;

// usb user callback funktions
USBD_Usr_cb_TypeDef USR_cb =
{
	USBD_USR_Init,
	USBD_USR_DeviceReset,
	USBD_USR_DeviceConfigured,
	USBD_USR_DeviceSuspended,
	USBD_USR_DeviceResumed,
	USBD_USR_DeviceConnected,
	USBD_USR_DeviceDisconnected
};

// usb callback functions
CDC_IF_Prop_TypeDef VCP_fops =
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

// COM-Port baudrate
static LINE_CODING linecoding =
{
	115200, /* baud rate*/
	0x00,   /* stop bits-1*/
	0x00,   /* parity - none*/
	0x08    /* nb. of bits 8*/
};

// usb desktriptor strings
USBD_DEVICE USR_desc =
{
  USBD_USR_DeviceDescriptor,
  USBD_USR_LangIDStrDescriptor,
  USBD_USR_ManufacturerStrDescriptor,
  USBD_USR_ProductStrDescriptor,
  USBD_USR_SerialStrDescriptor,
  USBD_USR_ConfigStrDescriptor,
  USBD_USR_InterfaceStrDescriptor
};

// usb deskriptor
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x02,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_OTG_MAX_EP0_SIZE,      /*bMaxPacketSize*/
    LOBYTE(USBD_VID),           /*idVendor*/
    HIBYTE(USBD_VID),           /*idVendor*/
    LOBYTE(USBD_PID),           /*idProduct*/
    HIBYTE(USBD_PID),           /*idProduct*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    USBD_CFG_MAX_NUM            /*bNumConfigurations*/
}; /* USB_DeviceDescriptor */


__ALIGN_BEGIN uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END =
{
  USB_SIZ_STRING_LANGID,
  USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING),
  HIBYTE(USBD_LANGID_STRING)
};

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// configure DM and DP pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);

	// enable clock for SYSCFG and USB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);
}

void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USB_OTG_BSP_uDelay (const uint32_t usec)
{
	uint32_t count = 0;
	const uint32_t utime = (120 * usec / 7);
	do
	{
		if ( ++count > utime )
		{
			return ;
		}
	}
	while (1);
}

void USB_OTG_BSP_mDelay (const uint32_t msec)
{
	USB_OTG_BSP_uDelay(msec * 1000);
}

void USBD_USR_Init(void)
{
}

void USBD_USR_DeviceReset(uint8_t speed )
{
  switch (speed)
  {
    case USB_OTG_SPEED_HIGH:
      break;

    case USB_OTG_SPEED_FULL:
      break;

    default:
      break;
  }
}

void USBD_USR_DeviceConfigured(void)
{
}

void USBD_USR_DeviceSuspended(void)
{
}

void USBD_USR_DeviceResumed(void)
{
}

void USBD_USR_DeviceConnected(void)
{
}

void USBD_USR_DeviceDisconnected(void)
{
}

uint8_t*  USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length)
{
	*length = sizeof(USBD_DeviceDesc);
	return USBD_DeviceDesc;
}


uint8_t*  USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length)
{
	*length =  sizeof(USBD_LangIDDesc);
	return USBD_LangIDDesc;
}


uint8_t*  USBD_USR_ProductStrDescriptor( uint8_t speed , uint16_t *length)
{
	USBD_GetString (USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}


uint8_t*  USBD_USR_ManufacturerStrDescriptor( uint8_t speed , uint16_t *length)
{
	USBD_GetString (USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}


uint8_t*  USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length)
{
	USBD_GetString (USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}


uint8_t*  USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length)
{
	USBD_GetString (USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}


uint8_t*  USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length)
{
	USBD_GetString (USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

static uint16_t VCP_Init(void)
{
	return USBD_OK;
}

static uint16_t VCP_DeInit(void)
{
	return USBD_OK;
}

static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
	switch (Cmd)
	{
		case SEND_ENCAPSULATED_COMMAND:
			break;
		case GET_ENCAPSULATED_RESPONSE:
			break;
		case SET_COMM_FEATURE:
			break;
		case GET_COMM_FEATURE:
			break;
		case CLEAR_COMM_FEATURE:
			break;
		case SET_LINE_CODING:
			linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
			linecoding.format = Buf[4];
			linecoding.paritytype = Buf[5];
			linecoding.datatype = Buf[6];
			break;
		case GET_LINE_CODING:
			Buf[0] = (uint8_t)(linecoding.bitrate);
			Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
			Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
			Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
			Buf[4] = linecoding.format;
			Buf[5] = linecoding.paritytype;
			Buf[6] = linecoding.datatype;
			break;
		case SET_CONTROL_LINE_STATE:
			break;
		case SEND_BREAK:
			break;
		default:
			break;
	}
	return USBD_OK;
}

static uint16_t VCP_DataTx (uint8_t* Buf, uint32_t Len)
{
	return USBD_OK;
}

static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
	uint32_t i;

	for (i = 0; i < Len; i++)
	{
		if(USBTxCounter<USB_BUFFER_SIZE)
			USBTxBuffer[USBTxCounter++]=Buf[i];
	}
	return USBD_OK;
}

void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

/* initialize usb connection */
void usb_init()
{
	USBD_Init(&USB_OTG_dev,
			USB_OTG_FS_CORE_ID,
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);
}

void usb_detach()
{
	DCD_DevDisconnect(&USB_OTG_dev);
}

/* get tmcl-command from usb interface (9 Bytes) */
uint8_t usb_getUSBCmd(uint8_t *cmd)
{
	if(USBTxCounter>=9)
	{
		uint32_t i;
		for(i=0; i<9; i++)
			cmd[i] = USBTxBuffer[i];

		USBTxCounter = 0;
		return true;
	}
	return false;
}

/* send data */
void usb_sendData(uint8_t *buffer, uint32_t size)
{
	uint32_t i;
	for(i=0; i<size; i++)
	{
		APP_Rx_Buffer[APP_Rx_ptr_in++] = buffer[i];
		if(APP_Rx_ptr_in >= APP_RX_DATA_SIZE)
			APP_Rx_ptr_in=0;
	}
}

#else
	#error "interrupt handler not defined for selected BOARD_CPU!"
#endif

#endif // USE_USB_INTERFACE
