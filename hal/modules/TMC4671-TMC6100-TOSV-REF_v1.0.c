/*
 * TMC4671-TMC6100-TOSV-REF_v1.0.c
 *
 *  Created on: 08.04.2020
 *      Author: ED
 */

#include "TMC4671-TMC6100-TOSV-REF_v1.0.h"
#include "BLDC.h"
#include "hal/system/SysTick.h"

#if DEVICE==TMC4671_TMC6100_TOSV_REF_V10

// general module settings
const char *VersionString="0020V103";

// ADC configuration
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001244C)
#define ADC1_CHANNELS		4
static volatile uint16_t ADC1Value[ADC1_CHANNELS];	// array for analog values (filled by DMA)

// ADC1
uint8_t	ADC_VOLTAGE = 3;
uint8_t ADC_MOT_TEMP = 4;

void tmcm_initModuleConfig()
{
	moduleConfig.baudrate 				= 7; // UART 115200bps
	moduleConfig.serialModuleAddress 	= 1;
	moduleConfig.serialHostAddress		= 2;
}

void tmcm_initMotorConfig()
{
	// firmware default values

	motorConfig[0].maximumCurrent 			= 3000;
	motorConfig[0].maxVelocity 				= 80000;
	motorConfig[0].acceleration				= 20000;
	motorConfig[0].maxPressure				= 50000;
	motorConfig[0].useVelocityRamp			= true;
	motorConfig[0].openLoopCurrent			= 1000;
	motorConfig[0].motorType				= TMC4671_THREE_PHASE_BLDC;
	motorConfig[0].motorPolePairs			= 2;
	motorConfig[0].commutationMode			= COMM_MODE_FOC_DISABLED;
	motorConfig[0].adc_I0_offset			= 33200;
	motorConfig[0].adc_I1_offset			= 33200;

	motorConfig[0].hallPolarity 			= 0;
	motorConfig[0].hallDirection			= 0;
	motorConfig[0].hallInterpolation		= 1;
	motorConfig[0].hallPhiEOffset			= 0;

	motorConfig[0].dualShuntFactor			= 135;

	motorConfig[0].shaftBit					= 1;

	motorConfig[0].pidTorque_P_param		= 300;
	motorConfig[0].pidTorque_I_param		= 1000;
	motorConfig[0].pidVelocity_P_param		= 500;
	motorConfig[0].pidVelocity_I_param		= 100;

	motorConfig[0].pidPressure_P_param		= 3000;
	motorConfig[0].pidPressure_I_param		= 3000;

	motorConfig[0].pidVolume_P_param		= 3000;
	motorConfig[0].pidVolume_I_param		= 3000;

	motorConfig[0].pwm_freq 				= 100000;

	motorConfig[0].tStartup					= 1000;
	motorConfig[0].tInhalationRise			= 500;
	motorConfig[0].tInhalationPause			= 1000;
	motorConfig[0].tExhalationFall			= 500;
	motorConfig[0].tExhalationPause			= 1500;
	motorConfig[0].pLIMIT					= 5000;
	motorConfig[0].pPEEP					= 1500;

	// init ramp generator
	tmc_linearRamp_init(&rampGenerator[0]);

	// tosv control
	tosv_init(&tosvConfig[0]);
}

void tmcm_updateConfig()
{
	uint32_t delay = systick_getTimer();
	while(abs(systick_getTimer()-delay) < 100){;}

	// === configure linear ramp generator
	rampGenerator[0].maxVelocity  = motorConfig[0].maxVelocity;
	rampGenerator[0].acceleration = motorConfig[0].acceleration;
	rampGenerator[0].rampEnabled  = motorConfig[0].useVelocityRamp;

	// use motor config to update tosv values with EEPROM stored values
	tosvConfig[0].tStartup 			= motorConfig[0].tStartup;
	tosvConfig[0].tInhalationRise 	= motorConfig[0].tInhalationRise;
	tosvConfig[0].tInhalationPause 	= motorConfig[0].tInhalationPause;
	tosvConfig[0].tExhalationFall 	= motorConfig[0].tExhalationFall;
	tosvConfig[0].tExhalationPause 	= motorConfig[0].tExhalationPause;
	tosvConfig[0].pLIMIT 			= motorConfig[0].pLIMIT;
	tosvConfig[0].pPEEP 			= motorConfig[0].pPEEP;

	// === configure TMC6200 ===
	tmc6200_writeInt(DEFAULT_DRV, TMC6200_GCONF, 0);	// normal pwm control
	tmc6200_writeInt(DEFAULT_DRV, TMC6200_DRV_CONF, 0);	// BBM_OFF and DRVSTRENGTH to weak

	// === configure TMC4671 ===

	// dummy readout
	tmc4671_readInt(DEFAULT_MC, TMC4671_MOTOR_TYPE_N_POLE_PAIRS);

	// Motor type &  PWM configuration
	tmc4671_writeInt(DEFAULT_MC, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, ((u32)motorConfig[0].motorType << TMC4671_MOTOR_TYPE_SHIFT) | motorConfig[0].motorPolePairs);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_POLARITIES, 0x00000000);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_MAXCNT, 0x00000F9F);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_SV_CHOP, 0x00000107);

	// ADC configuration
	tmc4671_writeInt(DEFAULT_MC, TMC4671_ADC_I_SELECT, 0x18000100);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_dsADC_MCLK_A, 0x10000000);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_dsADC_MCLK_B, 0x00000000);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_ADC_I0_SCALE_OFFSET, 0x01000000 | motorConfig[0].adc_I0_offset);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_ADC_I1_SCALE_OFFSET, 0x01000000 | motorConfig[0].adc_I1_offset);

	// hall configuration
	bldc_updateHallSettings(DEFAULT_MC);

	// PI configuration
	tmc4671_setTorqueFluxPI(DEFAULT_MC, motorConfig[0].pidTorque_P_param, motorConfig[0].pidTorque_I_param);
	tmc4671_setVelocityPI(DEFAULT_MC, motorConfig[0].pidVelocity_P_param, motorConfig[0].pidVelocity_I_param);

	// limit configuration
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PID_VELOCITY_LIMIT, motorConfig[0].maxVelocity * motorConfig[0].motorPolePairs);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_OPENLOOP_ACCELERATION, motorConfig[0].acceleration);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PIDOUT_UQ_UD_LIMITS, 0x7FFF);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS, 0x7FFF);
	tmc4671_setTorqueFluxLimit_mA(DEFAULT_MC, motorConfig[0].dualShuntFactor, motorConfig[0].maximumCurrent);

	// reset target values
	tmc4671_writeInt(DEFAULT_MC, TMC4671_UQ_UD_EXT, 0);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PIDIN_TORQUE_TARGET_FLUX_TARGET, 0);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PIDIN_VELOCITY_TARGET, 0);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_OPENLOOP_VELOCITY_TARGET, 0);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_OPENLOOP_ACCELERATION, 0x0000FFFF);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PIDIN_POSITION_TARGET, 0);
}

/* expected by the libTMC library for io_init()*/
void tmcm_initModuleSpecificIO()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// === enable port A ===
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// analog inputs port A
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // AIN2 | NTC
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	// outputs port A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_15;		// CS_Weasel | CS_Dragon | RUN_LED | ERROR_LED | EN_Weasel
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BSRR = GPIO_Pin_3;						// disable CS_Weasel
	GPIOA->BSRR = GPIO_Pin_8;						// disable CS_Dragon
	GPIOA->BSRR = GPIO_Pin_9;						// disable RUN_LED
	GPIOA->BRR = GPIO_Pin_10;						// disable ERROR_LED
	GPIOA->BSRR = GPIO_Pin_15;						// enable EN_Weasel

	// === enable port B ===
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// analog inputs port B
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // AIN1 | AIN0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	// outputs port B
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 		// CS_EEPROM
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BSRR = GPIO_Pin_12;						// disable CS_EEPROM

	// === enable port C ===
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // === enable port D ===
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_PD01, DISABLE);  // keep PD0 and PD1 as oscillator inputs
}

void tmcm_initModuleSpecificADC()
{
	// enable clock for ADC and DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// DMA configuration
	DMA_InitTypeDef DMAInit;
	DMA_DeInit(DMA1_Channel1);
	DMAInit.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
	DMAInit.DMA_MemoryBaseAddr = (u32)ADC1Value;
	DMAInit.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMAInit.DMA_BufferSize = ADC1_CHANNELS;
	DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMAInit.DMA_Mode = DMA_Mode_Circular;
	DMAInit.DMA_Priority = DMA_Priority_High;
	DMAInit.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMAInit);

	// enable DMA
	DMA_Cmd(DMA1_Channel1, ENABLE);

	// ADC configuration
	ADC_InitTypeDef ADCInit;
	ADCInit.ADC_Mode = ADC_Mode_Independent;
	ADCInit.ADC_ScanConvMode = ENABLE;
	ADCInit.ADC_ContinuousConvMode = ENABLE;
	ADCInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADCInit.ADC_DataAlign = ADC_DataAlign_Right;
	ADCInit.ADC_NbrOfChannel = ADC1_CHANNELS;
	ADC_Init(ADC1, &ADCInit);

	// select adc channels for ADC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_28Cycles5);// use ADC_AIN0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_28Cycles5);// use ADC_AIN1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_28Cycles5);// use ADC_AIN2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SampleTime_28Cycles5);// use ADC_MOT_TEMP

	ADC_ITConfig(ADC1, ADC_IT_EOC|ADC_IT_AWD|ADC_IT_JEOC, DISABLE);

	// enable ADC-DMA
	ADC_DMACmd(ADC1, ENABLE);

	// enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// calibrate ADC1 automatically
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	// start current measurement
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void tmcm_led_run_toggle()
{
	GPIOA->ODR ^= GPIO_Pin_9;
}

void tmcm_enableDriver(uint8_t motor)
{
	if (motor == DEFAULT_MC)
		GPIOA->BSRR = BIT15;
}

void tmcm_disableDriver(uint8_t motor)
{
	if (motor == DEFAULT_MC)
		GPIOA->BRR = BIT15;
}

uint8_t tmcm_getDriverState(uint8_t motor)
{
	if (motor == DEFAULT_MC)
		return (GPIOA->IDR & BIT15) ? 1:0;

	return 0;
}

void tmcm_enableCsWeasel(uint8_t motor)
{
	if (motor == DEFAULT_MC)
		GPIOA->BRR = GPIO_Pin_3;
}

void tmcm_disableCsWeasel(uint8_t motor)
{
	if (motor == DEFAULT_MC)
		GPIOA->BSRR = GPIO_Pin_3;
}

void tmcm_enableCsDragon(uint8_t motor)
{
	if (motor == DEFAULT_DRV)
		GPIOA->BRR = GPIO_Pin_8;
}

void tmcm_disableCsDragon(uint8_t motor)
{
	if (motor == DEFAULT_DRV)
		GPIOA->BSRR = GPIO_Pin_8;
}

void tmcm_enableCsMem()
{
	GPIOB->BRR = GPIO_Pin_12;
}

void tmcm_disableCsMem()
{
	GPIOB->BSRR = GPIO_Pin_12;
}

// RS242/RS485 configuration
void tmcm_setRS485ToSendMode(){}
void tmcm_setRS485ToReceiveMode(){}
uint8_t tmcm_isRS485Sending(){ return false; }

void tmcm_clearModuleSpecificIOPin(uint8_t pin)
{
	switch(pin)
	{
		case 0:
			GPIOA->BRR = GPIO_Pin_15; 	// EN_4671
			break;
	}
}

void tmcm_setModuleSpecificIOPin(uint8_t pin)
{
	switch(pin)
	{
		case 0:
			GPIOA->BSRR = GPIO_Pin_15; 	// EN_4671
			break;
	}
}

uint8_t tmcm_getModuleSpecificIOPinStatus(uint8_t pin)
{
	switch (pin)
	{
		case 0:
			return (GPIOA->IDR & GPIO_Pin_15) ? 1:0;  // EN_4671
			break;
	}
	return 0;
}

uint16_t tmcm_getModuleSpecificADCValue(uint8_t pin)
{
	switch(pin)
	{
		case 0:
			return ADC1Value[0];  // ADC_AIN0
			break;
		case 1:
			return ADC1Value[1];  // ADC_AIN1
			break;
		case 2:
			return ADC1Value[2];  // ADC_AIN2
			break;
		case 3:					  // ADC_VOLTAGE
			return (tmc4671_readFieldWithDependency(DEFAULT_MC, TMC4671_ADC_RAW_DATA, TMC4671_ADC_RAW_ADDR, 1, TMC4671_ADC_VM_RAW_MASK, TMC4671_ADC_VM_RAW_SHIFT) - VOLTAGE_OFFSET);
			break;
		case 4:
			return ADC1Value[3]; // ADC_MOT_TEMP
	}
	return 0;
}

#endif
