/**
  ******************************************************************************
  * @file    main.c
  * @author  Central LAB
  * @version V2.2.0
  * @date    23-December-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "MetaDataManager.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "HWAdvanceFeatures.h"
#include "PeakCounter.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define BLUEMSYS_N_BUTTON_PRESS 3
#define BLUEMSYS_CHECK_CALIBRATION ((uint32_t)0x12345678)
#define MAC_ADDRESS                 0x12, 0x34, 0x00, 0xE1, 0x80, 0x03
#define ADVERTISING_INTERVAL_INCREMENT (16)

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define STORE_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

#define STORE_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )
#define INT32_T(x)                 (int32_t)(x)
/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;

#ifdef STM32_SENSORTILE
  #ifdef OSX_BMS_ENABLE_PRINTF
    extern TIM_HandleTypeDef  TimHandle;
    extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
  #endif /* OSX_BMS_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Code for MotionFX integration - Start Section */
extern uint32_t osx_mfx_license[3][4];
/* Code for MotionFX integration - End Section */
/* Exported Variables -------------------------------------------------------------*/

float sensitivity;
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

osxMFX_calibFactor magOffset; 

uint32_t ConnectionBleStatus  =0;

uint32_t ForceReCalibration    =0;
uint32_t FirstConnectionConfig =0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef    TimCCHandle;
TIM_HandleTypeDef    TimEnvHandle;
TIM_HandleTypeDef    TimAudioDataHandle;

uint8_t bdaddr[6];

/* Table with All the known osx License */
MDM_knownOsxLicense_t known_OsxLic[]={
  
  /* Code for MotionFX integration - Start Section */
  {OSX_MOTION_FX,"Motion","osxMotionFX x9/x6 v1.0.7"},
  /* Code for MotionFX integration - End Section */
  {OSX_END,"LAST",""}/* THIS MUST BE THE LAST ONE */
};


/* Private variables ---------------------------------------------------------*/
uint8_t buff[128];

/* Code for MotionFX integration - Start Section */
static volatile uint32_t Quaternion      =0;
/* Code for MotionFX integration - End Section */
static unsigned char isCal = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);


static unsigned char SaveCalibrationToMemory(uint32_t *MagnetoCalibration);
static unsigned char ResetCalibrationInMemory(uint32_t *MagnetoCalibration);
static unsigned char ReCallCalibrationFromMemory(uint32_t *MagnetoCalibration);

static void InitTimers(void);

void AudioProcess(void);
void StartTimer(void);
tBleStatus Beacon_Init(void);
void BlueNRG_Init(void);
tBleStatus UpdateAdv(uint8_t * buff,uint8_t len);

/* Code for MotionFX integration - Start Section */
static void ComputeQuaternions(void);
/* Code for MotionFX integration - End Section */

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
static void SendAudioSourceLocalizationData(void);
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

uint32_t t_coin=0;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();
  InitTargetPlatform(TARGET_SENSORTILE);
  
  /* Init the Meta Data Manager */
  InitMetaDataManager((void *)known_OsxLic,MDM_DATA_TYPE_LIC,NULL);
  /* Enable all the osx Motion License found on Meta Data Manager */
  {
    int32_t Index=0;
    while(known_OsxLic[Index].LicEnum!=OSX_END) {
      MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[known_OsxLic[Index].LicEnum].Address;
      if(PayLoad->osxLicenseInitialized) {
        switch(known_OsxLic[Index].LicEnum) {
          
          /* Code for MotionFX integration - Start Section */
          case OSX_MOTION_FX:
            MCR_OSX_COPY_LICENSE_FROM_MDM(osx_mfx_license,PayLoad->osxLicense);
            MotionFX_License_init(PayLoad);
          break;
          default:
            /* Only for removing the GCC warning */
            OSX_BMS_PRINTF("Should never reach this point...\r\n");
            break;
        }
#ifdef OSX_BMS_LICENSE_H_FILE
      } else {
        switch(known_OsxLic[Index].LicEnum) {
          
          /* Code for MotionFX integration - Start Section */
          case OSX_MOTION_FX:
            MotionFX_License_init(PayLoad);
          break;
          /* Code for BlueVoice integration - End Section */
        }
      }
#else /* OSX_BMS_LICENSE_H_FILE */
      }  
#endif /* OSX_BMS_LICENSE_H_FILE */
      Index++;
    }
  }
#ifdef OSX_BMS_DEBUG_CONNECTION
  OSX_BMS_PRINTF("Debug Connection         Enabled\r\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */

  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();

  /* Initialize the BlueNRG HCI */
  HCI_Init();
    
  /* Reset BlueNRG hardware */
  BlueNRG_RST();
  
  BlueNRG_Init();
  
  Beacon_Init();
  
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    InitHWFeatures();
  }

  /* Set Accelerometer Full Scale to 8G */
  Set8GAccelerometerFullScale();

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);

  /* initialize timers */
  InitTimers();

  /* Control if the calibration is already available in memory */
  if(MDM_LicTable[OSX_MOTION_FX].Address) {
    MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_FX].Address;
    if(PayLoad->osxLicenseInitialized) {
      ReCallCalibrationFromMemory(PayLoad->ExtraData);    
	}
  }
  StartTimer();
  MDM_PayLoadLic_t *PayLoad;
  /* Code for MotionFX integration - Start Section */    
  /* Initialize MotionFX library */
  PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_FX].Address;
  if(PayLoad) {
     if((PayLoad->osxLicenseInitialized) & (TargetBoardFeatures.osxMotionFXIsInitalized==0)){
      MotionFX_manager_init();
      MotionFX_manager_start_9X();
    }
  }
  ResetCalibrationInMemory(PayLoad->ExtraData);
  /* Infinite loop */
  while (1){
    
      if(!TargetBoardFeatures.LedStatus) {
        if(!(HAL_GetTick()&0x3FF)) {
          LedOnTargetPlatform();
        }
      } else {
        if(!(HAL_GetTick()&0x3F)) {
          LedOffTargetPlatform();
        }
      }
      HCI_Process();
    /* Wait for Event */
    __WFI();
  }
}


/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
void Set4GAccelerometerFullScale(void)
{
  
  /* Set Full Scale to +/-4g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,4.0f);

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}
/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
void Set8GAccelerometerFullScale(void)
{
  
  /* Set Full Scale to +/-8g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,8.0f);

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  /* Code for MotionFX and MotionGR integration - Start Section */
  /* TIM1_CH1 toggling with frequency = 100Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
    
    ComputeQuaternions();
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimEnvHandle)) {

#ifdef STM32_SENSORTILE
#ifdef OSX_BMS_ENABLE_PRINTF
    } else if(htim == (&TimHandle)) {
      CDC_TIM_PeriodElapsedCallback(htim);
#endif /* OSX_BMS_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */
  }
}

uint32_t drumCount = 0;
uint8_t peek = 0;
/* Code for MotionFX integration - Star Section */
/* @brief  osxMotionFX Working function
 * @param  None
 * @retval None
 */
static void ComputeQuaternions(void)
{
  static int32_t calibIndex =0;
  static int32_t CounterFX  =0;
  static int32_t CounterEC  =0;
  SensorAxesRaw_t ACC_Value_Raw;
  SensorAxes_t GYR_Value;
  SensorAxes_t MAG_Value;

  /* Increment the Counter */
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC)) {
    CounterEC++;
  } else {
    CounterFX++;
  }

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);

  /* Read the Magneto values */
  BSP_MAGNETO_Get_Axes(TargetBoardFeatures.HandleMagSensor,&MAG_Value);

  /* Read the Gyro values */
  BSP_GYRO_Get_Axes(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);

  MotionFX_manager_run(ACC_Value_Raw,GYR_Value,MAG_Value);
     
  //需关闭校准，否则传感器板子受大力后会无数据输出一段时间

//  /* Check if is calibrated */
//  if(isCal!=0x01){
//    /* Run Compass Calibration @ 25Hz */
//    calibIndex++;
//    if (calibIndex == 4){
//      calibIndex = 0;
//      ACC_Value.AXIS_X = (int32_t)(ACC_Value_Raw.AXIS_X * sensitivity);
//      ACC_Value.AXIS_Y = (int32_t)(ACC_Value_Raw.AXIS_Y * sensitivity);
//      ACC_Value.AXIS_Z = (int32_t)(ACC_Value_Raw.AXIS_Z * sensitivity);
//      osx_MotionFX_compass_saveAcc(ACC_Value.AXIS_X,ACC_Value.AXIS_Y,ACC_Value.AXIS_Z);	/* Accelerometer data ENU systems coordinate	*/
//      osx_MotionFX_compass_saveMag(MAG_Value.AXIS_X,MAG_Value.AXIS_Y,MAG_Value.AXIS_Z);	/* Magnetometer  data ENU systems coordinate	*/            
//      osx_MotionFX_compass_run();
//
//      /* Control the calibration status */
//      isCal = osx_MotionFX_compass_isCalibrated();
//      if(isCal == 0x01){
//        /* Get new magnetometer offset */
//        osx_MotionFX_getCalibrationData(&magOffset);
//
//        /* Save the calibration in Memory */
//        {
//          MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_FX].Address;
//          SaveCalibrationToMemory(PayLoad->ExtraData);
//        }
//
//        /* Switch on the Led */
//        LedOnTargetPlatform();
//      }
//    }
//  }else {
//    calibIndex=0;
//  }

  /* Read the quaternions */
  osxMFX_output *MotionFX_Engine_Out = MotionFX_manager_getDataOUT();
 {

    /* Scaling quaternions data by a factor of 10000
      (Scale factor to handle float during data transfer BT) */
//
//    /* Save the quaternions values */
//    qi = MotionFX_Engine_Out->quaternion_9X[0];
//    qj = MotionFX_Engine_Out->quaternion_9X[1];
//    qk = MotionFX_Engine_Out->quaternion_9X[2];
//    qs = MotionFX_Engine_Out->quaternion_9X[3];
//    STORE_LE_16(buff+0,((int32_t)(qi * 10000)));
//    STORE_LE_16(buff+2,((int32_t)(qj * 10000)));
//    STORE_LE_16(buff+4,((int32_t)(qk * 10000)));
//    STORE_LE_16(buff+6,((int32_t)(qs * 10000)));
//    UpdateAdv(buff,8);
    
//    /* Save the quaternions values */
//    int32_t x = ((int32_t)(MotionFX_Engine_Out->rotation_9X[0] * 100));
//    int32_t y = ((int32_t)(MotionFX_Engine_Out->rotation_9X[1] * 100));
//    int32_t z = ((int32_t)(MotionFX_Engine_Out->rotation_9X[2] * 100));
//    STORE_LE_16(buff+0,x);
//    STORE_LE_16(buff+2,y);
//    STORE_LE_16(buff+4,z);
//    UpdateAdv(buff,6);
    
    
//    /* Save the quaternions values */
//    int32_t x = ((int32_t)(MotionFX_Engine_Out->rotation_9X[0] * 100));
//    int32_t y = ((int32_t)(MotionFX_Engine_Out->rotation_9X[1] * 100));
//    int32_t z = ((int32_t)(MotionFX_Engine_Out->quaternion_9X[3] * 10000));
//    STORE_LE_16(buff+0,x);
//    STORE_LE_16(buff+2,y);
//    STORE_LE_16(buff+4,z);
//    UpdateAdv(buff,6);
//    
    
//    int32_t x = ((int32_t)(MotionFX_Engine_Out->rotation_9X[0] * 100));

    int32_t x = ACC_Value_Raw.AXIS_X > ACC_Value_Raw.AXIS_Y ? ACC_Value_Raw.AXIS_X : ACC_Value_Raw.AXIS_Y;
    x = x > ACC_Value_Raw.AXIS_Z ? x : ACC_Value_Raw.AXIS_Z;

    int32_t y = ((int32_t)(MotionFX_Engine_Out->rotation_9X[2] * 100));
    int32_t z = ((int32_t)(MotionFX_Engine_Out->rotation_9X[0] * 100));
    STORE_LE_16(buff+0,drumCount);
    STORE_LE_16(buff+2,y);
    STORE_LE_16(buff+4,z);
    UpdateAdv(buff,6);
    /*
    if(x > 30000)
    {
      peek = 1;
    }else if(x < 10000 && peek == 1)
    {
      peek = 0;
      drumCount++;
    }
   */
   drumCount += PeakCounter(((int32_t)(MotionFX_Engine_Out->rotation_9X[2] * 100)),100,6,250);
#ifdef OSX_BMS_ENABLE_PRINTF
/*
      OSX_BMS_PRINTF("%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                     INT32_T(MotionFX_Engine_Out->rotation_9X[0]),
                     INT32_T(MotionFX_Engine_Out->rotation_9X[1]),
                     INT32_T(MotionFX_Engine_Out->rotation_9X[2]),
                     
                     INT32_T(MotionFX_Engine_Out->quaternion_9X[0] * 100),
                     INT32_T(MotionFX_Engine_Out->quaternion_9X[1] * 100),
                     INT32_T(MotionFX_Engine_Out->quaternion_9X[2] * 100),
                     INT32_T(MotionFX_Engine_Out->quaternion_9X[3] * 100)
                       );
*/    
    OSX_BMS_PRINTF("%d\t%d\r\n",((int32_t)(MotionFX_Engine_Out->rotation_9X[2] * 100)),((int32_t)(MotionFX_Engine_Out->rotation_9X[0] * 100)));
//    OSX_BMS_PRINTF("%d\r\n",((int32_t)(MotionFX_Engine_Out->rotation_9X[2] * 100)));
//   OSX_BMS_PRINTF("%d\t%d\t%d\t%d\r\n",drumCount,x,y,z);
//    OSX_BMS_PRINTF("9x:%d\t%d\t%d\r\n",quat_axes[QuaternionNumber].AXIS_X,quat_axes[QuaternionNumber].AXIS_Y,quat_axes[QuaternionNumber].AXIS_Z);
#endif /* OSX_BMS_ENABLE_PRINTF */
  }
}

/**
* @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to have TIM4 counter clock equal to 2 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2000) - 1); 
  
  /* Set TIM4 instance (Environmental)*/
  TimEnvHandle.Instance = TIM4;
  /* Initialize TIM4 peripheral */
  TimEnvHandle.Init.Period = ENV_UPDATE_MUL_100MS*200 - 1;
  TimEnvHandle.Init.Prescaler = uwPrescalerValue;
  TimEnvHandle.Init.ClockDivision = 0;
  TimEnvHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimEnvHandle) != HAL_OK) {
    /* Initialization Error */
  }

    /* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  /* Set TIM1 instance (Motion)*/
  TimCCHandle.Instance = TIM1;  
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Code for MotionFX and MotionGR integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

}
#ifdef USE_STM32F4XX_NUCLEO
#ifdef STM32_NUCLEO
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow:
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is
  clocked below the maximum system frequency, to update the voltage scaling value
  regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }
  
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
    Error_Handler();
  }
}
#endif /* STM32_NUCLEO */
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#ifdef STM32_NUCLEO
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;  /* Previous value = RCC_MSIRANGE_6; */
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6; /* Previous value = 1 */
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 4; /* Previous value = 2 */
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
}
#elif STM32_SENSORTILE
/**
* @brief  System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  
  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();
  
  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();
  
  /* Select MSI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    while(1);
  }
}
#endif /* STM32_NUCLEO */
#endif /* USE_STM32L4XX_NUCLEO */

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
    case BNRG_SPI_EXTI_PIN: 
      HCI_Isr();
    break;
  case LSM6DSM_INT2_PIN:
    break;
  }
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
     ex: OSX_BMS_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif

/**
 * @brief  Save the Magnetometer Calibration Values to Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
static unsigned char SaveCalibrationToMemory(uint32_t *MagnetoCalibration)
{
  unsigned char Success=1;

  /* Reset Before The data in Memory */
  Success = ResetCalibrationInMemory(MagnetoCalibration);

  if(Success) {
    void *temp;
    /* Store in RAM */
    MagnetoCalibration[0] = BLUEMSYS_CHECK_CALIBRATION;
    MagnetoCalibration[1] = (uint32_t) magOffset.magOffX;
    MagnetoCalibration[2] = (uint32_t) magOffset.magOffY;
    MagnetoCalibration[3] = (uint32_t) magOffset.magOffZ;

    temp = ((void *) &(magOffset.magGainX));
    MagnetoCalibration[4] = *((uint32_t *) temp);
    temp = ((void *) &(magOffset.magGainY));
    MagnetoCalibration[5] = *((uint32_t *) temp);
    temp = ((void *) &(magOffset.magGainZ));
    MagnetoCalibration[6] = *((uint32_t *) temp);
    temp = ((void *) &(magOffset.expMagVect));
    MagnetoCalibration[7] = *((uint32_t *) temp);
    NecessityToSaveMetaDataManager=1;
  }

  return Success;
}

/**
 * @brief  Reset the Magnetometer Calibration Values in Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
static unsigned char ResetCalibrationInMemory(uint32_t *MagnetoCalibration)
{
  /* Reset Calibration Values in RAM */
  unsigned char Success=1;
  int32_t Counter;

  for(Counter=0;Counter<8;Counter++)
    MagnetoCalibration[Counter]=0xFFFFFFFF;
  NecessityToSaveMetaDataManager=1;
  
  return Success;
}

/**
 * @brief  Check if there are a valid Calibration Values in Memory and read them
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallCalibrationFromMemory(uint32_t *MagnetoCalibration)
{
  /* ReLoad the Calibration Values from RAM */
  unsigned char Success=1;

  if(MagnetoCalibration[0]== BLUEMSYS_CHECK_CALIBRATION) {
    void *temp;
    magOffset.magOffX    = (signed short) MagnetoCalibration[1];
    magOffset.magOffY    = (signed short) MagnetoCalibration[2];
    magOffset.magOffZ    = (signed short) MagnetoCalibration[3];
    temp= (void *)&(MagnetoCalibration[4]);
    magOffset.magGainX = *((float *)temp);
    temp= (void *)&(MagnetoCalibration[5]);
    magOffset.magGainY = *((float *)temp);
    temp= (void *)&(MagnetoCalibration[6]);
    magOffset.magGainZ = *((float *)temp);
    temp= (void *)&(MagnetoCalibration[7]);
    magOffset.expMagVect = *((float *)temp);

    /* Set the Calibration Structure */
    osx_MotionFX_setCalibrationData(&magOffset);
    OSX_BMS_PRINTF("Magneto Calibration Read\r\n");

    /* Control the calibration status */
    isCal = osx_MotionFX_compass_isCalibrated();
  } else {
    OSX_BMS_PRINTF("Magneto Calibration Not present\r\n");
    isCal=0;
  }

  return Success;
}
void StartTimer(void)
{
      /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
          /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
    }
}
/**
 * @brief  Initialize the BlueNRG
 *
 * @param  None
 * @retval None
 */
void BlueNRG_Init(void)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t SERVER_BDADDR[] = { MAC_ADDRESS };

  uint8_t  hwVersion;
  uint16_t fwVersion;
  uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1; 
  }
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  SERVER_BDADDR);

  if (ret != BLE_STATUS_SUCCESS)
  {
    __asm("nop");
  }

  ret = aci_gatt_init();

  if (ret != BLE_STATUS_SUCCESS)
  {
    __asm("nop");
  }

  if (bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    __asm("nop");
  }

  ret = aci_hal_set_tx_power_level(1,4);

  if (ret != BLE_STATUS_SUCCESS)
  {
    __asm("nop");
  }

}

tBleStatus Beacon_Init(void)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  /* Disable scan response. */
  hci_le_set_scan_resp_data(0, NULL);

  uint16_t AdvertisingInterval = (1 * ADVERTISING_INTERVAL_INCREMENT / 10);

  /* Put the device in a non-connectable mode. */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND,                          /*< Advertise as non-connectable, undirected. */
                                 AdvertisingInterval, AdvertisingInterval, /*< Set the advertising interval as 700 ms (0.625 us increment). */
                                 PUBLIC_ADDR, NO_WHITE_LIST_USE,           /*< Use the public address, with no white list. */
                                 0, NULL,                                  /*< Do not use a local name. */
                                 0, NULL,                                  /*< Do not include the service UUID list. */
                                 0, 0);                                    /*< Do not set a slave connection interval. */

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Remove the TX power level advertisement (this is done to decrease the packet size). */
  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  uint8_t service_data[] =
  {
    23,                                                                      /*< Length. */
    AD_TYPE_SERVICE_DATA,                                                    /*< Service Data data type value. */
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
  };

  uint8_t service_uuid_list[] =
  {
    3,                                                                      /*< Length. */
    AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST,                                    /*< Complete list of 16-bit Service UUIDs data type value. */
    0xAA, 0xFE                                                              /*< 16-bit Eddystone UUID. */
  };

  uint8_t flags[] =
  {
    2,                                                                      /*< Length. */
    AD_TYPE_FLAGS,                                                          /*< Flags data type value. */
    (FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED) /*< BLE general discoverable, without BR/EDR support. */
  };

  /* Update the service data. */
  ret = aci_gap_update_adv_data(sizeof(service_data), service_data);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Update the service UUID list. */
  ret = aci_gap_update_adv_data(sizeof(service_uuid_list), service_uuid_list);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Update the adverstising flags. */
  ret = aci_gap_update_adv_data(sizeof(flags), flags);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  return ret;
}
tBleStatus UpdateAdv(uint8_t * buff,uint8_t len)
{  
  tBleStatus ret = BLE_STATUS_SUCCESS;
  uint8_t service_data[] =
  {
    23,                                                                      /*< Length. */
    AD_TYPE_SERVICE_DATA,                                                    /*< Service Data data type value. */
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
    0x00,  
  }; 
  memcpy(&service_data[2],buff,len);
  memset(&service_data[2+len],0,22-len);
  /* Update the service data. */
  ret = aci_gap_update_adv_data(sizeof(service_data), service_data);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }
  return ret;
}
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
