#include "bme.h"
#include "cmsis_os.h" // FreeRTOS kütüphanesi (osDelay için)

// --- YARDIMCI SPI FONKSİYONLARI ---
static void BME280_WriteReg(BME280_Handle_t *dev, uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = reg & 0x7F; // Write Mask
    data[1] = value;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->hspi, data, 2, 100);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void BME280_ReadReg(BME280_Handle_t *dev, uint8_t reg, uint8_t *buffer, uint8_t len) {
    uint8_t tx = reg | 0x80; // Read Mask
    
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->hspi, &tx, 1, 100);
    HAL_SPI_Receive(dev->hspi, buffer, len, 100);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// Kalibrasyon Okuma (Eski kodundaki işlemler struct içine alındı)
static void BME280_Read_Calibration_Data(BME280_Handle_t *dev) {
    uint8_t calib_data1[26];
    uint8_t calib_data2[7];

    BME280_ReadReg(dev, BME280_CALIB1_START, calib_data1, 26);
    BME280_ReadReg(dev, BME280_CALIB2_START, calib_data2, 7);

    dev->T1 = (calib_data1[1] << 8) | calib_data1[0];
    dev->T2 = (calib_data1[3] << 8) | calib_data1[2];
    dev->T3 = (calib_data1[5] << 8) | calib_data1[4];
    dev->P1 = (calib_data1[7] << 8) | calib_data1[6];
    dev->P2 = (calib_data1[9] << 8) | calib_data1[8];
    dev->P3 = (calib_data1[11] << 8) | calib_data1[10];
    dev->P4 = (calib_data1[13] << 8) | calib_data1[12];
    dev->P5 = (calib_data1[15] << 8) | calib_data1[14];
    dev->P6 = (calib_data1[17] << 8) | calib_data1[16];
    dev->P7 = (calib_data1[19] << 8) | calib_data1[18];
    dev->P8 = (calib_data1[21] << 8) | calib_data1[20];
    dev->P9 = (calib_data1[23] << 8) | calib_data1[22];
    dev->H1 = calib_data1[25];
    dev->H2 = (calib_data2[1] << 8) | calib_data2[0];
    dev->H3 = calib_data2[2];
    dev->H4 = (calib_data2[3] << 4) | (calib_data2[4] & 0x0F);
    dev->H5 = (calib_data2[5] << 4) | (calib_data2[4] >> 4);
    dev->H6 = calib_data2[6];
}

uint8_t BME280_Init(BME280_Handle_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    
    // LPF Başlangıç Ayarı
    dev->lpf_alpha = 0.15f; // %15 Yeni Veri, %85 Eski Veri (İdeal yumuşatma)
    dev->filtered_alt = 0.0f;

    uint8_t chip_id = 0;
    BME280_ReadReg(dev, BME280_REG_ID, &chip_id, 1);
    
    if (chip_id != 0x60 && chip_id != 0x58) return 0; // Hata
    return 1; // Başarılı
}

void BME280_Setup(BME280_Handle_t *dev, uint8_t osrs_p, uint8_t osrs_t, uint8_t osrs_h, uint8_t mode, uint8_t filter, uint8_t standby) {
    BME280_Read_Calibration_Data(dev);
    
    BME280_WriteReg(dev, BME280_REG_RESET, 0xB6);
    osDelay(100); // KÖTÜ KODDAKİ HAL_Delay GİTTİ, RTOS UYUMLU BEKLEME GELDİ!
    
    uint8_t ctrl_hum_val = osrs_h;
    uint8_t config_val   = (standby << 5) | (filter << 2);
    uint8_t ctrl_meas_val= (osrs_t << 5) | (osrs_p << 2) | mode;
    
    BME280_WriteReg(dev, BME280_REG_CTRL_HUM, ctrl_hum_val);
    BME280_WriteReg(dev, BME280_REG_CONFIG, config_val);
    BME280_WriteReg(dev, BME280_REG_CTRL_MEAS, ctrl_meas_val);
}

// Bütün veriyi okuyup hesabı yapan ve LPF'yi uygulayan ana fonksiyon
void BME280_Read_All(BME280_Handle_t *dev) {
    uint8_t RawData[8];
    BME280_ReadReg(dev, BME280_REG_PRESS_MSB, RawData, 8);

    int32_t PresRaw = ((uint32_t)RawData[0] << 12) | ((uint32_t)RawData[1] << 4) | ((uint32_t)RawData[2] >> 4);
    int32_t TempRaw = ((uint32_t)RawData[3] << 12) | ((uint32_t)RawData[4] << 4) | ((uint32_t)RawData[5] >> 4);
    int32_t HumRaw  = ((uint32_t)RawData[6] << 8) | ((uint32_t)RawData[7]);

    // --- 1. SICAKLIK HESABI ---
    int32_t var1, var2;
    var1 = ((((TempRaw >> 3) - ((int32_t)dev->T1 << 1))) * ((int32_t)dev->T2)) >> 11;
    var2 = (((((TempRaw >> 4) - ((int32_t)dev->T1)) * ((TempRaw >> 4) - ((int32_t)dev->T1))) >> 12) * ((int32_t)dev->T3)) >> 14;
    dev->t_fine = var1 + var2; 
    dev->temperature = ((dev->t_fine * 5 + 128) >> 8) / 100.0f;

    // --- 2. BASINÇ HESABI ---
    int64_t varp1, varp2, p;
    varp1 = ((int64_t)dev->t_fine) - 128000;
    varp2 = varp1 * varp1 * (int64_t)dev->P6;
    varp2 = varp2 + ((varp1 * (int64_t)dev->P5) << 17);
    varp2 = varp2 + (((int64_t)dev->P4) << 35);
    varp1 = ((varp1 * varp1 * (int64_t)dev->P3) >> 8) + ((varp1 * (int64_t)dev->P2) << 12);
    varp1 = (((((int64_t)1) << 47) + varp1)) * ((int64_t)dev->P1) >> 33;

    if (varp1 == 0) dev->pressure = 0;
    else {
        p = 1048576 - PresRaw;
        p = (((p << 31) - varp2) * 3125) / varp1;
        varp1 = (((int64_t)dev->P9) * (p >> 13) * (p >> 13)) >> 25;
        varp2 = (((int64_t)dev->P8) * p) >> 19;
        p = ((p + varp1 + varp2) >> 8) + (((int64_t)dev->P7) << 4);
        dev->pressure = (float)p / 256.0f; 
    }

    // --- 3. LPF UYGULANMIŞ İRTİFA HESABI ---
    if(dev->pressure > 0) {
        float raw_altitude = 44330.0f * (1.0f - powf(dev->pressure / 101325.0f, 1.0f / 5.255f));
        
        // IIR LPF (Exponential Moving Average) Uygulaması
        if(dev->filtered_alt == 0.0f) {
            dev->filtered_alt = raw_altitude; // İlk okumada filtreyi başlat
        } else {
            // Y_yeni = (Alpha * X_yeni) + ((1 - Alpha) * Y_eski)
            dev->filtered_alt = (dev->lpf_alpha * raw_altitude) + ((1.0f - dev->lpf_alpha) * dev->filtered_alt);
        }
        dev->altitude = dev->filtered_alt; // Çıktıyı güncelle
    } else {
        dev->altitude = 0;
    }
}