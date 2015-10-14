/**
  ******************************************************************************
  * @file    ADC_DMA/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
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
#include "stm32f0xx.h"
#include "stm32f0_discovery.h"

/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup ADC_DMA
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address                0x40012440
#define SEVEN_SEGMENT_SCAN_INTERVAL    20

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t TempSensVoltmv = 0, VrefIntVoltmv = 0;
__IO uint16_t RegularConvData_Tab[2];
__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0;

GPIO_InitTypeDef        GPIO_InitStructure;
static __IO uint32_t TimingDelay;
static __IO uint16_t value_seg; 
static __IO uint16_t scanInterval;
static __IO uint16_t value_digits;
static __IO uint8_t digi_4;
static __IO uint8_t digi_3;
static __IO uint8_t digi_2;
static __IO uint8_t digi_1;
static __IO uint16_t senceCount;
static __IO uint16_t oldLightADC;
static __IO uint8_t fallingEdgeFlag;
static __IO uint8_t risingEdgeFlag;

/* Private function prototypes -----------------------------------------------*/
void ADC1_CH_DMA_Config(void);
static void SysTickConfig(void);
void SevenSegValueSet(uint8_t value, uint8_t digit);
void ShowValue(uint16_t value);
void SevenSegScan(void);
void TimingDelay_Decrement(void);

/* Private functions ---------------------------------------------------------*/


uint8_t seven_seg_digits[10][7] = { { 1,1,1,1,1,1,0 },  // = 0
                                    { 0,1,1,0,0,0,0 },  // = 1
                                    { 1,1,0,1,1,0,1 },  // = 2
                                    { 1,1,1,1,0,0,1 },  // = 3
                                    { 0,1,1,0,0,1,1 },  // = 4
                                    { 1,0,1,1,0,1,1 },  // = 5
                                    { 1,0,1,1,1,1,1 },  // = 6
                                    { 1,1,1,0,0,0,0 },  // = 7
                                    { 1,1,1,1,1,1,1 },  // = 8
                                    { 1,1,1,0,0,1,1 }   // = 9
                             };

uint16_t aa[10] = { 0x003F,0x0006,0x005B,0x004F,0x0066,0x006D,0x007D,0x0007,0x007F,0x0067};


void TimingDelay_Decrement(void)
{
  if (TimingDelay > 0)
  {
    TimingDelay--;
  }
}

void TimeDelay(uint16_t ms)
{
  TimingDelay = ms;
  
  while (TimingDelay > 0);
}
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */
  scanInterval = SEVEN_SEGMENT_SCAN_INTERVAL;
    /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  /* Configure PC10 and PC11 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
//  SevenSegValueSet(5,0);

  GPIO_SetBits(GPIOC, GPIO_Pin_8);

//  GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_9);
  
  /* SysTick configuration ---------------------------------------------------*/
  SysTickConfig();
  
  /* ADC1 channel with DMA configuration */
  ADC1_CH_DMA_Config();
  ShowValue(0);
  oldLightADC = 0;
  senceCount = 0;
  /* Infinite loop */
  while (1)
  { 
//    SevenSegValueSet(2,2);
//    TimeDelay(10);
//    SevenSegValueSet(3,1);
//    TimeDelay(10);
//    SevenSegValueSet(8,3);
    TimeDelay(10);
//    TimeDelay(2);
//    SevenSegValueSet(4,0);
//    TimeDelay(40);
    /* Test DMA1 TC flag */
    while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ); 
    
    /* Clear DMA TC flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
    
    /* Convert temperature sensor voltage value in mv */
  //  ShowValue(RegularConvData_Tab[0] %10000);
    
    if(fallingEdgeFlag == 0)
    {
      if(oldLightADC > (RegularConvData_Tab[0] + 50))
      {
        fallingEdgeFlag++;
      }
      else
        oldLightADC = RegularConvData_Tab[0];
    }
    else
    {
      if(oldLightADC > (RegularConvData_Tab[0] + 50))
      {
        fallingEdgeFlag++;
        oldLightADC = RegularConvData_Tab[0];
      }
      else if ((oldLightADC +30) < RegularConvData_Tab[0])
      {
        fallingEdgeFlag = 0;
        senceCount++;
        ShowValue(senceCount %10000);
      }
    }

    /* Convert Vref voltage value in mv */
    //VrefIntVoltmv  = (uint32_t)((RegularConvData_Tab[1]* 3300) / 0xFFF);  
    
  }
}


void SevenSegScan(void)
{
  if ((scanInterval % 5) == 0)
  {
    if(value_seg == 0)
      SevenSegValueSet( 0, 0);
    else
    {
      if((scanInterval == 20) && (value_digits > 3))
        SevenSegValueSet( digi_4, scanInterval / 5 -1);
      else if ((scanInterval == 15)&& (value_digits > 2))
        SevenSegValueSet( digi_3, scanInterval / 5 -1);
      else if ((scanInterval == 10)&& (value_digits > 1))
        SevenSegValueSet( digi_2, scanInterval / 5 -1);
      else if (scanInterval == 5)
        SevenSegValueSet( digi_1, scanInterval / 5 -1);
    }
  }
    
  if( scanInterval > 0)
    scanInterval--;
  else
    scanInterval = SEVEN_SEGMENT_SCAN_INTERVAL;
}


void ShowValue(uint16_t value)
{
  uint16_t temp_value;
  value_seg = value;
  
  if ( value_seg > 999)
    value_digits = 4;
  else if (  value_seg > 99)
    value_digits = 3;
  else if (  value_seg > 9)
    value_digits = 2;
  else
    value_digits = 1;
  
  if(value > 0)
  {
    digi_1 = value % 10;
    temp_value = value - value % 10;
    
    digi_2 = (temp_value % 100) / 10;
    temp_value = value - value % 100;
    
    digi_3 = (temp_value % 1000) / 100;
    temp_value = value - value % 1000;
    
    digi_4 = temp_value / 1000;
  }  
}

void SevenSegValueSet(uint8_t value, uint8_t digit)
{
  uint8_t i;
  uint16_t pin = GPIO_Pin_3;
  
  switch(digit)
  {
    case 3:
      GPIO_ResetBits(GPIOC, GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
      GPIO_SetBits(GPIOC, GPIO_Pin_12);
    break;
    case 2:
      GPIO_ResetBits(GPIOC, GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12);
      GPIO_SetBits(GPIOC, GPIO_Pin_11);
    break;
    case 1:
      GPIO_ResetBits(GPIOC, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12);
      GPIO_SetBits(GPIOC, GPIO_Pin_10);
    break;
    case 0:
    default:
      GPIO_ResetBits(GPIOC, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
      GPIO_SetBits(GPIOC, GPIO_Pin_9);
    break;
  }
  
  
  GPIO_Write(GPIOB, ((~(aa[value % 10]))<< 3));
/*  
  for( i = 0; i < 7; i++)
  {
    if( seven_seg_digits[value][i] == 0)
      GPIO_SetBits(GPIOB, pin);
    
    pin = pin << 1;
  }  
  */
}

/**
  * @brief  ADC1 channel with DMA configuration
  * @param  None
  * @retval None
  */
void ADC1_CH_DMA_Config(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  DMA_InitTypeDef     DMA_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  
   /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* Configure ADC Channel1 analog input */

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* ADC1 DeInit */  
  ADC_DeInit(ADC1);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
  
  /* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* DMA1 Channel1 enable */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC DMA request in circular mode */
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
  
  /* Enable ADC_DMA */
  ADC_DMACmd(ADC1, ENABLE);  
  
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
  ADC_Init(ADC1, &ADC_InitStructure); 
 
  /* Convert the ADC1 temperature sensor  with 55.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_1,   ADC_SampleTime_55_5Cycles);       //ADC_Channel_TempSensor ,
  ADC_TempSensorCmd(ENABLE);
  
  /* Convert the ADC1 Vref  with 55.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint , ADC_SampleTime_55_5Cycles); 
  ADC_VrefintCmd(ENABLE);
  
  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADCEN falg */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
}

/**
  * @brief  Configure a SysTick Base time to 10 ms.
  * @param  None
  * @retval None
  */
static void SysTickConfig(void)
{
  /* Setup SysTick Timer for 10ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }

  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
