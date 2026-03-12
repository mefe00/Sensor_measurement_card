#include "can_manager.h"
#include <stdio.h>

// mian.c'de bulunna handle yapısını kullanmak için extern ediyoruz (İPTAL EDİLDİ - Modülerlik için parametre olarak alıyoruz)
// extern CAN_HandleTypeDef hcan1; 

// Gönderme işlemi için gerekli yapılar (İPTAL EDİLDİ - Race Condition önlemek için fonksiyon içine alındı)
// static CAN_TxHeaderTypeDef pHeader;
// static uint32_t pTxDatabox; 

// Burada CAN hattı için geçerli filtre ayarlarını yapıyoruz.
void CAN_Config_Init(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0; // 14 tane filtre bankası var, 0.sını kullanıyoruz.
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Maskeleme modu (IP Maskesi gibi)
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    
    // --- MANTIK ---
    // FilterId: "Aradığım numara şu"
    // FilterMask: "Numaranın hangi haneleri tutmalı?"
    // Biz Mask'ı 0x0000 yapıyoruz. Yani "Hiçbir hane umurumda değil, hepsini al".
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Gelen mesajları FIFO0 kutusuna at
    sFilterConfig.FilterActivation = ENABLE; // Filtreyi çalıştır
    sFilterConfig.SlaveStartFilterBank = 14; // Tek CAN kullandığımız için önemsiz

    // Filtreyi Yükle
    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        // Hata Yönetimi (Buraya while(1) koyabilirsin test için)
    }

    // --- KRİTİK NOKTA ---
    // Sadece Init yetmez, "Start" demeden CAN hattı elektriklenmez.
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        // Hata
    }

    // Kesmeleri (Interrupt) Aç: Mesaj gelince işlemciye düdük çal
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        // Hata
    }
}


uint8_t CAN_Send_Message(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len) {
    // DEĞİŞİKLİK: Race condition (çakışma) olmaması için değişkenler local (yerel) tanımlandı.
    CAN_TxHeaderTypeDef pHeader;
    uint32_t pTxMailbox; // pTxDatabox yazım hatası düzeltildi

    pHeader.StdId = id;
    pHeader.IDE = CAN_ID_STD;
    pHeader.RTR = CAN_RTR_DATA;
    pHeader.DLC = len;
    pHeader.TransmitGlobalTime = DISABLE;

    // --- GÜNCELLEME: BOŞ KUTU BEKLEME (NON-BLOCKING) ---
    // FreeRTOS kullanacağımız için while(1) ile işlemciyi burada hapsetmiyoruz!
    // Eğer kutular doluysa direkt başarısız dönüyoruz, tekrar deneme işini Task'e bırakıyoruz.
    uint32_t freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
    
    if (freeLevel == 0) {
        return 0; // Hat çok yoğun, kutular dolu! (Task içinde osDelay yapıp tekrar denenecek)
    }

    // hcan1 yerine parametre olarak gelen hcan kullanıldı (Modülerlik)
    if (HAL_CAN_AddTxMessage(hcan, &pHeader, data, &pTxMailbox) != HAL_OK) {
        return 0; // Donanımsal hata
    }
    return 1; // Başarılı
}


// Donanımdan mesaj geldiğinde otomatik çalışan fonksiyon
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Mesajı kutudan al
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        // BURASI ÇOK ÖNEMLİ:
        // Gelen veriyi burada İŞLEMİYORUZ (Çünkü burası Interrupt, sistemi dondurur).
        // Örnek: Eğer gelen ID == 0x500 ise motoru durdur vs. mantığını 
        // burada kurmak yerine mesajı FreeRTOS Queue (Kuyruk) yapısına fırlatacağız.
        
        // Şimdilik debug için boş bırakıyoruz veya bir LED yakabiliriz.
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Mesaj gelince LED yak
        
        // İLERİDE EKLENECEK KOD: 
        // xQueueSendFromISR(CAN_RxQueue, &rxData, &xHigherPriorityTaskWoken);
    }
}

// main.c içinde yani bu dosyaların kullanımı:
// Öncelikle can_manager.h dosyasını include et 
// Hazırladığımız CAN_Config_Init ile CAN hattını kur ve başlat
// Eğer FreeRTOS bir yapı ile kullanacaksanız Task içinde şu şekilde kullanılacaktır.
/*void StartCanTask(void *argument)
{
  // USER CODE BEGIN StartCanTask 
  uint8_t txData[8] = {0}; // 8 Byte'lık boş paket
  uint32_t counter = 0;

  for(;;)
  {
    // Veriyi Doldur: [Sayaç (4 byte) | 0 | 0 | 0 | 0]
    // Bit kaydırma (Shift) ile 32-bit sayıyı 8-bit parçalara bölüyoruz
    txData[0] = (counter >> 24) & 0xFF; // En büyük parça
    txData[1] = (counter >> 16) & 0xFF;
    txData[2] = (counter >> 8)  & 0xFF;
    txData[3] = (counter)       & 0xFF; // En küçük parça

    // ID: 0x100 (Sistem Durumu) olarak gönder (hcan1 referans olarak eklendi)
    if(CAN_Send_Message(&hcan1, CAN_ID_SYSTEM_STATUS, txData, 8)) {
        printf("CAN Gonderildi: %lu\r\n", counter);
    } else {
        // Kutular doluysa veya hata varsa burada biraz bekle ve tekrar dene
        printf("CAN Hatasi! (Mailbox Dolu Olabilir)\r\n");
    }

    counter++;
    osDelay(100); // 100ms bekle (Saniyede 10 mesaj)
  }
  // USER CODE END StartCanTask 
}
  */