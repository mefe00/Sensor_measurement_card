#ifndef INC_NEO_M8N_H_
#define INC_NEO_M8N_H_

#include "stm32f4xx_hal.h"

// UBX Sync Byte'ları
#define UBX_SYNC_CHAR_1  0xB5
#define UBX_SYNC_CHAR_2  0x62
#define UBX_NAV_CLASS    0x01
#define UBX_NAV_PVT_ID   0x07

// NOT:
// Bunun asıl sebebi gelen verilerin arasında boşluk olmamasını sağlamak.
// Çünkü derleyici performans için struct'ları hizalamaya çalışır ve bu da bazı durumlarda veri kaymasına neden olabilir.
// Eğer bunu yaparsa enlem boylam verisine girer ve okunan bütün değerler yanlış okunur.
// Bunun için pragma komutu kullandık.
// Struct yapısınındada normalde işimize yaramayacak olsa bile bütün verileri aldık. boşluk kalmamasını garanti almak için ve ayrıştırıken kolay olması için.
#pragma pack(push, 1) 
typedef struct {
    uint32_t iTOW;      // GPS Zamanı (ms)
    uint16_t year;      
    uint8_t  month, day, hour, min, sec;
    uint8_t  valid;     // Fix geçerli mi?
    uint32_t tAcc;      
    int32_t  nano;      
    uint8_t  fixType;   // 0=Yok, 2=2D, 3=3D Fix
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;     // Bağlı uydu sayısı
    int32_t  lon;       // Boylam (Derece * 1e7)
    int32_t  lat;       // Enlem (Derece * 1e7)
    int32_t  height;    // Elipsoid Yüksekliği (mm)
    int32_t  hMSL;      // Deniz Seviyesi Yüksekliği (mm)
    uint32_t hAcc;      // Yatay Hassasiyet (mm) -> KALMAN İÇİN KRİTİK!
    uint32_t vAcc;      // Dikey Hassasiyet (mm) -> KALMAN İÇİN KRİTİK!
    int32_t  velN;      // Kuzey Hızı (mm/s)
    int32_t  velE;      // Doğu Hızı (mm/s)
    int32_t  velD;      // Aşağı Hızı (mm/s)
    int32_t  gSpeed;    // 2D Yer Hızı (mm/s)
    int32_t  headMot;   // Hareket Yönü (Derece * 1e5)
    uint32_t sAcc;      // Hız Hassasiyeti (mm/s)
    uint32_t headAcc;   // Yön Hassasiyeti (Derece * 1e5)
    uint16_t pDOP;      // Konum Geometrisi Çarpanı
    uint8_t  reserved1[6];
    int32_t  headVeh;   
    int16_t  magDec;    
    uint16_t magAcc;    
} UBX_NAV_PVT_t;
#pragma pack(pop)

// Sistemde birden fazla GPS kullananlar için (bizim gibi) bu struct'ı kullanarak her GPS için ayrı bir yapı oluşturabiliriz.
typedef struct {
    UART_HandleTypeDef *huart;
    UBX_NAV_PVT_t data;     // Çözülen anlamlı veriler burada toplanacak
    uint8_t is_fixed;       // GPS bağlandı mı?
    
    // Arka plan Parser (Ayrıştırıcı) durum değişkenleri
    uint8_t rx_buffer[100]; // DMA'in veriyi dolduracağı ham havuz
    uint8_t state;          
    uint16_t payload_len;
    uint16_t payload_counter;
    uint8_t class_id;
    uint8_t msg_id;
    uint8_t ck_a;
    uint8_t ck_b;
} NEO_M8N_Handle_t;

// Kullanacağımız fonksiyonlar
void NEO_M8N_Init(NEO_M8N_Handle_t *hgps, UART_HandleTypeDef *huart);
void NEO_M8N_ParseBuffer(NEO_M8N_Handle_t *hgps, uint16_t length);

#endif /* INC_NEO_M8N_H_ */