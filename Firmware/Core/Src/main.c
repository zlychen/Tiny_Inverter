
  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @Author         : whutzf2010
  ******************************************************************************

  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
uint16_t ADCValue_VIN ;
uint16_t ADCValue_VACL;
uint16_t ADCValue_VACN;
uint16_t ADCValue_IIN;
uint16_t ADCValue_IAC;

float f32_VIN;
float f32_VIN_filter;
float f32_VIN_filtered;
float f32_VAC;
float f32_VAC_filter;
float f32_VAC_filtered;
float f32_IIN;
float f32_IAC;
float f32_IAC_filter;
float f32_IAC_filtered;

float f32_VAC_Rms[50];  //10KHZ/100HZ=100
int16_t f32_VAC_Rms_Counter;
float f32RMSValue;
float f32SUMValue;

float f32_VDC_Set;
float f32_VRms_Set;
float f32_IAC_Ref;
float f32_VAC_Set;
float f32_VAC_Set_Step;
float f32_VAC_Set_Update;
float f32_IAC_Set;
float f32_IAC_Set_Step;
float f32_IAC_Set_Update;
double f32_VAC_Gain;

uint16_t UsartTxValue;
uint16_t UsartRxValue;

uint16_t   aADCxConvertedValues[ADC_BUFFER_SIZE];



/* Private function prototypes -----------------------------------------------*/
static void Data_Init(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);


//**************************************************************//
//					Digital Power Main Loop						//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

int main(void)
{

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  SystemClock_Config();

  MX_DMA_Init();
  MX_ADC1_Init();
  MX_COMP2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  Data_Init();
  MX_GPIO_Init();
  MX_TIM1_Init();

  while (1)
  {}

}


//**************************************************************//
//						Digital Power Data Initialize											//
//								whutzf2010																				//
//								2023.July.17th 																//
//**************************************************************//

static void Data_Init(void)
{
//-------------- Mode Selection --------------//		
		u16Mode = InvertOnGrid;//InvertOffGrid;

//		u16VacState = POS1; //Totem-pole-state
//		u16OnGridState = Ready;
	
//-------- OnGrid Parameter Initialize --------------//
		f32_VDC_Set = 15.0;
	
	
		SOGIPLL.k = 1.41;
		SOGIPLL.w0 = 314.1593; //2*pi*50
		SOGIPLL.Ts = 1/(float)TIM2_FREQ;
		SOGIPLL.lamda = (float)(0.5*SOGIPLL.w0*SOGIPLL.Ts);
		SOGIPLL.x = (float)(2*SOGIPLL.k*SOGIPLL.w0*SOGIPLL.Ts);
		SOGIPLL.y = (float)(SOGIPLL.w0*SOGIPLL.Ts*SOGIPLL.w0*SOGIPLL.Ts);
		SOGIPLL.b0 = (float)(SOGIPLL.x/(SOGIPLL.x+SOGIPLL.y+4));
		SOGIPLL.a1 = (float)(8-2*SOGIPLL.y)/(SOGIPLL.x+SOGIPLL.y+4);
		SOGIPLL.a2 = (float)(SOGIPLL.x-SOGIPLL.y-4)/(SOGIPLL.x+SOGIPLL.y+4);
		SOGIPLL.u = 0.0;
		SOGIPLL.alpha0 =0.0;
		SOGIPLL.alpha1 =0.0;
		SOGIPLL.alpha2 =0.0;
		SOGIPLL.beta0 = 0.0;
		SOGIPLL.beta1 = 0.0;
		SOGIPLL.beta2 = 0.0;
		
		SOGIPLL.d = 0.0;
		SOGIPLL.q = 0.0;
		SOGIPLL.theta = 0.0;
		SOGIPLL.kp = 0.2;
		SOGIPLL.ki = 2.0;
		
		
		OnGridPID.Kp_Vol = 0.00000025;                              
		OnGridPID.Ki_Vol = 0.00000025;                                                                                                                                                                                                                                                                                                                              2;
		
		OnGridPR.Ts = 1/(float)TIM2_FREQ;
		OnGridPR.Kp = 0.15;
		OnGridPR.Kr = 7.0;
		OnGridPR.wc = 5.0;
		OnGridPR.w0 = 314.1593; //2*PI*50
		
		OnGridPR.A1 = 2*OnGridPR.Ts;
		OnGridPR.A2 = (2.0*OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0 -8.0)/(OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0+4.0*OnGridPR.wc*OnGridPR.Ts+4.0);
		OnGridPR.A3 = (OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0+4.0-4.0*OnGridPR.Ts*OnGridPR.wc)/(OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0+4.0*OnGridPR.Ts*OnGridPR.wc+4.0);
		OnGridPR.A4 = (4.0*OnGridPR.Kp+OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0+4.0*OnGridPR.Ts*OnGridPR.wc*OnGridPR.Kp+4.0*OnGridPR.Ts*OnGridPR.wc*OnGridPR.Kr)/(OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0+4.0*OnGridPR.Ts*OnGridPR.wc+4.0);
		OnGridPR.A5 = (2.0*OnGridPR.Kp*OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0-8.0*OnGridPR.Kp)/(OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0+4.0*OnGridPR.Ts*OnGridPR.wc+4.0);
		OnGridPR.A6 = (4.0*OnGridPR.Kp+OnGridPR.Kp*OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0-4.0*OnGridPR.Ts*OnGridPR.wc*OnGridPR.Kp-4.0*OnGridPR.Ts*OnGridPR.wc*OnGridPR.Kr)/(OnGridPR.Ts*OnGridPR.Ts*OnGridPR.w0*OnGridPR.w0+4.0*OnGridPR.Ts*OnGridPR.wc+4.0);
		
		OnGridDQ.Err0_Volt = 0.0;
		OnGridDQ.Err1_Volt = 0.0;
		OnGridDQ.Out0_Volt = 0.0;
		OnGridDQ.Out1_Volt = 0.0;
		OnGridDQ.Set_Volt = 25.0;  //
		OnGridDQ.Feedback_Volt = 0.0;

		OnGridDQ.Err0_D = 0.0;
		OnGridDQ.Err1_D = 0.0;
		OnGridDQ.Out0_D = 0.0;
		OnGridDQ.Out1_D = 0.0;
		OnGridDQ.Set_D = 0.0;
		OnGridDQ.Feedback_D = 0.0;
		
		OnGridDQ.Err0_Q = 0.0;
		OnGridDQ.Err1_Q = 0.0;
		OnGridDQ.Out0_Q = 0.0;
		OnGridDQ.Out1_Q = 0.0;
		OnGridDQ.Set_Q = 0.0;
		OnGridDQ.Feedback_Q = 0.0;
			
//----------- OffGrid Control Parameter ------------------//
		f32_VRms_Set = 15.50;
		
		f32_VAC_Set = (float)(f32_VRms_Set*1.414);
		f32_VAC_Set_Step = 0.1; //softstart can not too slow 
		f32_VAC_Set_Update = 0.0;

		OffGridPID.Err0_Vol = 0.0;
		OffGridPID.Err1_Vol = 0.0;		
		OffGridPID.Out0_Vol = 0.0;
		OffGridPID.Out1_Vol = 0.0;		
//	PID.Kp_Vol = 0.002;
//	PID.Ki_Vol = 0.001;
		
		OffGridPID.Err0_Cur = 0.0;
		OffGridPID.Err1_Cur = 0.0;		
		OffGridPID.Out0_Cur = 0.0;
		OffGridPID.Out1_Cur = 0.0;	
//	PID.Kp_Cur = 0.4;
//	PID.Ki_Cur = 0.4;

//-------- SPWM Parameter Initialize --------------//
		TIM2_Counter = 0;		
		TIM2_Delay_Counter = 0;
	
		UsartTxValue = 0xfb; //how to transfer 16-bit data.?
	  f32VacFreqCounter = 0.0;
	  u16VacFreqCounter = 0;
    VacTimer = 0.0;
	  f32VacVal = 0;
		f32VacSin = 0;;
	  VacPeakVal = INVERT_PWM - 1;
		f32_VAC_Gain = 1.0;

// --------- ADC Filter Parameter ------//

		f32_VAC_filter = 0.0;
		f32_VAC_filtered = 0.0;
		f32_VAC_Rms_Counter = 0;
		f32RMSValue = 0.0;
		f32SUMValue = 0.0;
		
		f32MaxValueBefore = 0.0;
		f32MaxValue = 0.0;
		f32MinValueBefore = 0.0;
		f32MinValue = 0.0;

	
//---------VOFA uart proctol initializa -------//
	//VOFA 上位机串尾为协议号，仅限JustFloat模式下	
		USARTBufferTx[12] = 0X00;
		USARTBufferTx[13] = 0X00;
		USARTBufferTx[14] = 0X80;
		USARTBufferTx[15] = 0x7f;
		
}

//**************************************************************//
//				Digital Power System Clock Configuration		//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_SYSCLK);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK2);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSRC_PLL_DIV_1);
  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_1);
}


static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /**ADC1 GPIO Configuration
  PC0   ------> ADC1_IN6
  PC1   ------> ADC1_IN7
  PC2   ------> ADC1_IN8
  PC3   ------> ADC1_IN9
  PA1   ------> ADC1_IN2
 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);



	/* ADC1 Init */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_AUTOWAIT;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
	//2023.8.20.12:03 Change ADC Trigger Source TIM1-CH3 --> TIM2 
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM2_TRGO;//LL_ADC_REG_TRIG_EXT_TIM1_CH3;//LL_ADC_REG_TRIG_SOFTWARE;
//	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_CH3;//LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;//LL_ADC_REG_CONV_CONTINUOUS;//LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;//LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT; //ADC1& ADC2 dual mode
	ADC_CommonInitStruct.MultiDMATransfer = LL_ADC_DMA_REG_REGULAR_DATA; //ADC1& ADC2 dual mode
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Analog WatchDog 1
  */
  LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_REG);
  LL_ADC_ConfigAnalogWDThresholds(ADC1, LL_ADC_AWD1, 0, 0);
  LL_ADC_DisableIT_AWD1(ADC1);

  /** Configure Regular Channel //ADC1.IN2 -PIN15
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2); 
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2,LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel //ADC1.IN6 -PIN8
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel //ADC1.IN7 -PIN9
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel //ADC1.IN8 - PIN10
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_8);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel //ADC1.IN9 -PIN11
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_9);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */
  LL_ADC_Enable(ADC1);
	LL_ADC_StartCalibration(ADC1,ADC_SINGLEDIFF_CALIB_START_MASK);  				//????
  //while (LL_ADC_IsCalibrationOnGoing(ADC1));   //??????
  /* USER CODE END ADC1_Init 2 */
  LL_ADC_REG_StartConversion(ADC1);  
}

//**************************************************************//
//					Digital Power ADC Configuration				//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  LL_COMP_InitTypeDef COMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**COMP2 GPIO Configuration
  PA7   ------> COMP2_INP
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  COMP_InitStruct.PowerMode = LL_COMP_POWERMODE_HIGHSPEED;
  COMP_InitStruct.InputPlus = LL_COMP_INPUT_PLUS_IO1;
  COMP_InitStruct.InputMinus = LL_COMP_INPUT_MINUS_DAC1_CH1;
  COMP_InitStruct.InputHysteresis = LL_COMP_HYSTERESIS_NONE;
  COMP_InitStruct.OutputSelection = LL_COMP_OUTPUT_TIM1_BKIN2;
  COMP_InitStruct.OutputPolarity = LL_COMP_OUTPUTPOL_INVERTED;
  COMP_InitStruct.OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE;
  LL_COMP_Init(COMP2, &COMP_InitStruct);
  LL_COMP_SetCommonWindowMode(__LL_COMP_COMMON_INSTANCE(COMP2), LL_COMP_WINDOWMODE_DISABLE);
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

//**************************************************************//
//				Digital Power DAC Configuration					//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**DAC GPIO Configuration
  PA4   ------> DAC_OUT1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC channel OUT1 config
  */
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  LL_DAC_DisableTrigger(DAC, LL_DAC_CHANNEL_1);
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

//**************************************************************//
//				Digital Power PWM Configuration				//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  //NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  //NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = INVERT_PWM-1; //////
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
	//---------CH2 PWM MODE 1-------------//
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.CompareValue = PWM_DUTY; /////
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
	//----------CH3 PWM MODE 2----------//
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
	//TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;//LL_TIM_OCPOLARITY_HIGH;
  //TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;//LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_ITR1);
  LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_DISABLED);
  LL_TIM_DisableIT_TRIG(TIM1);
  LL_TIM_DisableDMAReq_TRIG(TIM1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_ENABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = DT_600NS; /////
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  //TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  //TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  //TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_ENABLE;
  //TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  //TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  //TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PB1   ------> TIM1_CH3N
  PA9   ------> TIM1_CH2
  PA10   ------> TIM1_CH3
  PA12   ------> TIM1_CH2N
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_12;
  //GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;//LL_GPIO_MODE_INPUT;//LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  LL_TIM_CC_EnableChannel( TIM1 , LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableAllOutputs(TIM1); //2023.8.26  HF PWM	ENABLE SWITCH	
  LL_TIM_EnableCounter(TIM1);	

  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);//????

}

//**************************************************************//
//					Digital Power Interrupt	 Configuration		//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  //LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;  //1khz -Interrupt
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = PWM_5KHZ-1; //2kHz -Interrupt
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  //TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  //TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  //TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_ENABLE;
  //TIM_OC_InitStruct.CompareValue = PWM_2KHZ*0.5; /////
  //TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  //TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  //TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  //TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  //LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  //LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  //LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  //LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  //LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_ITR1);
  //LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_DISABLED);
  //LL_TIM_DisableIT_TRIG(TIM1);
  //LL_TIM_DisableDMAReq_TRIG(TIM1);
  //LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  //LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  //LL_TIM_DisableMasterSlaveMode(TIM1);
  //LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  //LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  //TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_ENABLE;
  //TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE;
  //TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  //TIM_BDTRInitStruct.DeadTime = DT_500NS; /////
  //TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  //TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  //TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  //TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_ENABLE;
  //TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  //TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  //TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
  LL_TIM_BDTR_Init(TIM2, &TIM_BDTRInitStruct);	
	//LL_TIM_CC_EnableChannel( TIM2 , LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH3);
  //LL_TIM_EnableAllOutputs(TIM2);		
  LL_TIM_EnableCounter(TIM2);	
	
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_EnableIT_UPDATE(TIM2);//????
}


//**************************************************************//
//				Digital Power USART Configuration				//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 Interrupt NVIC */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = BAUDRATE; ////
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;

  LL_USART_Init(USART2, &USART_InitStruct);
  //LL_USART_DisableIT_CTS(USART2);
  //LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  //LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableDMAReq_TX(USART2);////// 2022.11.22 zfan Important!
  //LL_USART_EnableDirectionTx(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

//**************************************************************//
//					Digital Power DMA Configuration				//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  //-----ADC1 --- DMA1 CH1---CONFIG -----------------//
  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
	  /* ADC1 DMA Init */ 
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD); //16-BIT
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD); //16-BIT
	
	/* Set DMA transfer addresses of source and destination */
	LL_DMA_ConfigAddresses(DMA1,
						   LL_DMA_CHANNEL_1,
						   LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
						   (uint32_t)&aADCxConvertedValues,
						   LL_DMA_DIRECTION_PERIPH_TO_MEMORY);//??DMA,?DMA?ADC1?????
		/* Set DMA transfer size */
	LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,5); //2022.11.23 zfan 5*16=80  fill 5 not 80 
	//LL_DMA_SetPeriphAddress(DMA1,LL_DMA_CHANNEL_1,LL_ADC_DMA_GetRegAddr(ADC1,LL_ADC_DMA_REG_REGULAR_DATA));
	//LL_DMA_SetMemoryAddress(DMA1,LL_DMA_CHANNEL_1,(uint32_t)&adcBuf[0]);
 
	/* Enable the DMA transfer */
	LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);//??DMA??
					 
   //-----------USART2 -- DMA1 CH7--CONFIG---------------//
  NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 2));
  NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	  /* USART2 DMA Init */ 
  //LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_CIRCULAR); // continous sending 2022.11.23 zfan
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_INCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_NOINCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE); //8-BIT
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);  //8-BIT
	
	/* Set DMA transfer addresses of source and destination */
	LL_DMA_ConfigAddresses(DMA1,
						   LL_DMA_CHANNEL_7,
						   (uint32_t)&USARTBufferTx,
						   LL_USART_DMA_GetRegAddr(USART2,LL_USART_DMA_REG_DATA_TRANSMIT),
						   LL_DMA_DIRECTION_PERIPH_TO_MEMORY);////2022.11.23 zfan Truth is memory to periph!! 
	/* Set DMA transfer size */
	LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_7,16); // 2022.11.23 zfan  4*8=32 fill 4 not 32
	//LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
 
	/* Enable the DMA transfer */
	LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_7);//??DMA??	

}

//**************************************************************//
//					Digital Power GPIO Configuration			//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
//  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); ///LED1~LED3
//	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13); ///LED1~LED3
//	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); ///LED1~LED3
	
  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14);  

  //B12, B14 LED
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_14; //LED1|LED3
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //B13, LED
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13; //LED2
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //A9, A12 --LF Bridge
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_12;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);  
//  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);   
  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

//**************************************************************//
//				Digital Power Other Configuration				//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{-
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

