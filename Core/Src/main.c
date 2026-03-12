/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
// ===================================================================== //
// ===================================================================== //
// ===================================================================== //
// Task_IMU == çok yüksek öncelik (SPI + DMA ve 200-400 Hz civarı)
// Task_GPS == yüksek öncelik (USART + DMA ve 10 Hz civarı)
// Task_CAN == orta öncelik (Interrupt bazlı ve 100 Hz civarı)
// Task_BME == düşük öncelik (SPI + DMA ve 20 Hz civarı)

// FAİLSAFE DURUMLARI:
// 1-) GPS kaybı: FreeRTOS yapısı ile bu durumda GPS Fix bayrağı düşecek ve GPS Task'ı atlanıp IMU verisi ile hareket sağlanacak. Aynı zamanda CAN üzerinden GPS_LOST loglanacak. Bu 2 GPS'de gittiği durumda.
// 2-) Haberleşme hattı kitlenmesi: SPI veya I2C hatları düşerse bunu algılamak için Timeout sayacı kullanılır. MCU Timeout süresi aşıldığında sensörün gücünü kesip tekrar açacak (reset).
// 3-) Kartın belirlenemeyen bir durumda donması ve hataya düşmesi halinde Watchdog Timer devreye girecek ve kartı resetleyecektir. Bu sayede kartın kendini kurtarması sağlanır.
// ===================================================================== //



/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalman_factory_measurement.h"
#include "kalman_factory_filter.h"
#include "neo-m8n.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// =========================================================================== //
// FİLTRE TANIMLAMALARI //
KALMAN_FACTORY_FILTER(alt_filter, 3);          // 3 durumlu X vektörü (z (basınç sensöründen alınan irtifa değeri), v (z eksenideki IMU'nun ölçtüğü hız), a (z eksenindeki IMU'dan alınan ivme) )
KALMAN_FACTORY_MEASUREMENT(bme_meas, 3, 1);    // BME280 ölçümü (1 boyutlu) [1 0 0] 
KALMAN_FACTORY_MEASUREMENT(imu_meas, 3, 1);    // MPU9250 Z ölçümü (1 boyutlu) [0 0 1]

/* =========================================================================== */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern NEO_M8N_Handle_t gps_1;
extern NEO_M8N_Handle_t gps_2;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
// =========================================================================== //
// FİLTRE İÇİN GEREKLİ FONKİYON TANIMLAMALARI // 

void Init_Altitude_Filter(float dt) {
    // F Matrisi (Fizik Kuralları) - Satır satır tek boyutlu diziye yazılır
    alt_filter.F.data[0] = 1.0f; alt_filter.F.data[1] = dt;   alt_filter.F.data[2] = 0.5f * dt * dt;
    alt_filter.F.data[3] = 0.0f; alt_filter.F.data[4] = 1.0f; alt_filter.F.data[5] = dt;
    alt_filter.F.data[6] = 0.0f; alt_filter.F.data[7] = 0.0f; alt_filter.F.data[8] = 1.0f;
    
    // H Matrisleri (Sensörlerin baktığı yerler)
    bme_meas.H.data[0] = 1.0f; bme_meas.H.data[1] = 0.0f; bme_meas.H.data[2] = 0.0f; // Sadece Konum
    imu_meas.H.data[0] = 0.0f; imu_meas.H.data[1] = 0.0f; imu_meas.H.data[2] = 1.0f; // Sadece İvme
    
    // R Matrisleri (Sensör Gürültüleri - Tuning burada yapılır)
    bme_meas.R.data[0] = 2.5f;   // BME280'e az güven
    imu_meas.R.data[0] = 0.1f;   // İvmeölçere çok güven
}

// 3. ADIM: FreeRTOS Task'inde Çalışan Ana Döngü
void Task_Filter_Update(void *pvParameters) {
    while(1) {
        // Fiziksel olarak bir adım ileri git (Predict)
        kalman_predict(&alt_filter);
        
        // MPU9250'den ivme geldiyse filtreyi güncelle (Update)
        if (IMU_Data_Ready()) {
            imu_meas.z.data[0] = MPU9250_Get_Z_Accel(); 
            kalman_correct(&alt_filter, &imu_meas); // Kalman Kazancı (K) burada hesaplanır!
        }
        
        // BME280'den irtifa geldiyse filtreyi güncelle (Update)
        if (BME280_Data_Ready()) {
            bme_meas.z.data[0] = BME280_Get_Altitude();
            kalman_correct(&alt_filter, &bme_meas);
        }
        
        // SONUÇ: Kaymak gibi filtrelenmiş irtifa ve dikey hız!
        float temiz_irtifa = alt_filter.x.data[0];
        float temiz_hiz    = alt_filter.x.data[1];
    }
}
// =========================================================================== //


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// =========================================================================== //
// ========================================================================== //
// GPS VERİLERİNİN DMA İLE ALINDIĞI CALLBACK FONKSİYONU //
// DMA UART Idle kesmesi düştüğünde donanım buraya girer
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Hangi GPS'ten veri geldiğini UART kanalından anlıyoruz
    if (huart->Instance == USART1) {
        NEO_M8N_ParseBuffer(&gps_1, Size); // Byte'ları çöz
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_1.rx_buffer, 100); // Tekrar dinlemeye başla
    }
    else if (huart->Instance == USART2) {
        NEO_M8N_ParseBuffer(&gps_2, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_2.rx_buffer, 100);
    }
}
// =========================================================================== //
// ========================================================================== //

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
