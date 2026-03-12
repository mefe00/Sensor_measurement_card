#ifndef BME280_H_
#define BME280_H_

#include "stm32f4xx_hal.h"
#include <math.h>

// ---------------------------------------------------------------------------
// REGISTER VE AYAR TANIMLARI (Senin dosyanla birebir aynı)
// ---------------------------------------------------------------------------
#define BME280_REG_ID              0xD0
#define BME280_REG_RESET           0xE0
#define BME280_REG_CTRL_HUM        0xF2
#define BME280_REG_STATUS          0xF3
#define BME280_REG_CTRL_MEAS       0xF4
#define BME280_REG_CONFIG          0xF5
#define BME280_REG_PRESS_MSB       0xF7

#define BME280_CALIB1_START        0x88
#define BME280_CALIB2_START        0xE1

// --- NESNE YÖNELİMLİ (OOP) SENSÖR YAPISI ---
// Global değişkenleri sildik, hepsini bu objenin içine aldık.
typedef struct {
    // Donanım Ayarları
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    // LPF (Düşük Geçiren Filtre) Ayarları
    float lpf_alpha;      // Filtre katsayısı (0.0 ile 1.0 arası. Örn: 0.1 çok yumuşatır)
    float filtered_alt;   // Filtrelenmiş irtifa hafızası

    // Sensör Kalibrasyon Verileri (Bosch formülleri için)
    uint16_t T1, P1;
    int16_t  T2, T3, P2, P3, P4, P5, P6, P7, P8, P9, H2, H4, H5;
    uint8_t  H1, H3;
    int8_t   H6;
    int32_t  t_fine;

    // Çıktılar
    float temperature;
    float pressure;
    float humidity;
    float altitude; // LPF'den geçmiş, kaymak gibi irtifa
} BME280_Handle_t;

// Fonksiyon Prototipleri (Artık hepsine objemizi veriyoruz)
uint8_t BME280_Init(BME280_Handle_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void BME280_Setup(BME280_Handle_t *dev, uint8_t osrs_p, uint8_t osrs_t, uint8_t osrs_h, uint8_t mode, uint8_t filter, uint8_t standby);
void BME280_Read_All(BME280_Handle_t *dev);

#endif


// Kulanımı
//değişkeleride exter n et kütüphnleri ekle
/* USER CODE END Header_Task_BME_Update */
/* void Task_BME_Update(void const *argument)
{
  /* USER CODE BEGIN Task_BME_Update */
  /* // 1. Sensörü Başlat
  if (BME280_Init(&bme_sensor, &hspi1, GPIOA, GPIO_PIN_4)) {
      printf("BME280 Basariyla Bulundu!\r\n");
  }

  // 2. Sensör Ayarlarını Yap (Oversampling x16, Normal Mod)
  BME280_Setup(&bme_sensor, 0x05, 0x05, 0x00, 0x03, 0x04, 0x00);

 
  for(;;)
  {
      // 3. Sensörü Oku ve Filtrele
      BME280_Read_All(&bme_sensor);
      
      // Float'ları tam sayı ve küsürat olarak ikiye ayırıyoruz
      int sicaklik_tam = (int)bme_sensor.temperature;
      int sicaklik_kusurat = (int)((bme_sensor.temperature - sicaklik_tam) * 100);
      
      int basinc_tam = (int)bme_sensor.pressure;
      int basinc_kusurat = (int)((bme_sensor.pressure - basinc_tam) * 100);
      
      int irtifa_tam = (int)bme_sensor.altitude;
      int irtifa_kusurat = (int)((bme_sensor.altitude - irtifa_tam) * 100);
      
      // %d ile birleştirerek basıyoruz (Sıfır eklemeleri için %02d kullanıyoruz)
      printf("Sicaklik: %d.%02d C | Basinc: %d.%02d Pa | LPF Irtifa: %d.%02d m\r\n", 
             sicaklik_tam, sicaklik_kusurat, 
             basinc_tam, basinc_kusurat, 
             irtifa_tam, irtifa_kusurat);
      osDelay(100); // Saniyede 10 kere (10Hz) oku
  }*/
  /* USER CODE END Task_BME_Update */
  /* 
}
*/