#include "neo-m8n.h"
#include <string.h>

// ======================================= DATASHEET ======================================== //
// DATASHEET: https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
// Sayfa 167 civarında UBX protkolü hakkında bilgi almaya başlayabilirsiniz.
// ======================================== END ============================================= //

// ================================ YAPILANDIRMA ============================================ //

// USART'ları açtıktan sonra DMA yapılandırması için:
// 1-) Add butonuna tıkla
// 2-) USARTx_RX seç
// 3-) Priority (önem sırası) High seç yada nasıl istiyorsan
// 4-) Mode Normal seç (Burada IDLE kesmesi kullandığımız için circular seçilmemesi lazım yoksa CPU'da buffer'a yazmaya çalışır)
// 5-) Data Width: Peripheral: Byte, Memory: Byte
// 6-) IDLE için kesme lazım NVIC'e gel kesmeyi aç (glogal interrupt)
// 7-) FreeRTOS kullanıyorsan (ben kullanacağım) USART DMA'nın önceiği FreeRTOS'tan yüksek olursa sistemi kilitler
// 7-) Bunu System Core kısmından NVIC bölümünden hangi interruptın aktifse bunların priprity değerini FreeRTOS değerinden yüksek yap. 
// 7-) Mesela 5 yap herhalde FreeRTOS 0-4 arasındaymış.

// =========================================== END ========================================== //

// ================================ KULLANIMI ============================================ //
/* 
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
*/
// =========================================== END ========================================== //

// NOT: Belki enum'u ilk kez duymuş olabilirsiniz. Kısaca açıklaması:
// enum aslında bir tür tanımlama aracıdır ve içine isimlendirilmiş sabitler koymamıza olanak tanır.
// Örneğin burada GPS verisini ayrıştırırken hangi aşamada olduğumuzu takip etmek için bir durum makinesi (state machine) oluşturduk ve bu enum ile o durumları isimlendirdik.
// Bu sayede kodun okunabilirliği artar ve hangi durumda ne yapıldığını anlamak kolaylaşır.
// Bu enum içinde tanımlananlar aslında state machine yapımızın birer durumu temsil eder ve enum yapısı gereği sıra sıra 0,1,2,.... diye gider.
// UBX protokolü ile okuma yapıyoruz ve bu protokolün yapısı gereği önce 2 adet sync byte'ı gelmeli, sonra class ve id byte'ları, sonra payload uzunluğu, sonra payload ve en son checksum byte'ları gelmelidir.
// Bunların hepsi aslında birer durumdur ve bekleme affetmez bu yüzden state machine yapısı kullanarak her gelen byte'ı anında işleyebiliriz ve işlemci hiçbir zaman boşta kalmaz, sürekli olarak gelen veriyi ayrıştırır ve anlamlı hale getirir.
// Bu 92 byte'lık veri yapısına u-blox'un datasheet'i üzerinden sayfadan bulabilirsiniz. Sayfa 168
enum {
    WAIT_SYNC1 = 0,  // Senkronizasyon byte'ı 1'i bekle. Bu karakter bekleriz == 0xB5
    WAIT_SYNC2,      // Senkronizasyon byte'ı 2'yi bekle. Bu karakter bekleriz == 0x62 1. gelir 2. gelmezse yanlış alarmdır.
    WAIT_CLASS,      // Class Kimliğidir. UBX protokolünde mesajların türleri vardır ve türlerinde kimlikleri. Gelen mesagın türünü buluruz. 
    WAIT_ID,         // Kimliği bulduktan sonra bu kimliğe ait ID'yi bulmamız lazım. UBX protokolünde her mesajın ama her kimliğin daha spesifik bir tanımlaması vardır ve bu ID'dir.
    WAIT_LEN1,       // Gelen mesaj hakkında bize uzunluk bilgisini verecek olan byte'lardır. WAIT_LEN1 bu 16-bitlik uzunluk bilgisinin (LSB) ilk byte'nını okuduğumuz yerdir.
    WAIT_LEN2,       // WAIT_LEN2 ise bu uzunluk bilgisinin (MSB) ikinci byte'ını okuduğumuz yerdir. Bu iki byte'ı birleştirerek gelen payload'un kaç byte olduğunu öğreniriz.
    GET_PAYLOAD,     // Asıl veri burada olur. Önceki iki veri paketinden aldığımız uzunluk bilgisini burada şu şekilde kullanırız: Bu veri paketinin dolmasını beklerken. Yani WAIT_LEN1 ve 2'den gelen bilgiden oluşan uzunluk bilgisi ile o uzunluk dolana kadar beklememiz lazım yoksa veri kaçırırız.
    GET_CK_A,        // Checksum A'yı bekle. Checksum'lar gelen verinin doğruluğunu kontrol etmek için kullanılır. UBX protokolünde checksum hesaplama algoritması Fletcher Algorithm'dır ve bu algoritmaya göre gelen verinin her byte'ını işleyerek iki adet checksum byte'ı hesaplarız ve bunları gelen checksum byte'larıyla karşılaştırırız. Eğer aynıysa veri doğru gelmiştir, değilse veri hatalı gelmiştir ve çöpe atılır.
    GET_CK_B         // Buda MSB kısmı ( Bu arada veri gürültü ile bozulmuş olabilir Checksum bunu doğrular)
};

// NOT: Burada state machine kullanmamızın sebebi işlemciyi 1 ms bile kaybetmemek aslında.
// Çünkü çok fazla ağır hesaplama içeren uygulamalrda işlemci 1 ms bile durmamalı ve zaten bizim uygulamamız çok ağır.
// State machine yapısı sayesinde her gelen byte'ı anında işleyebiliriz ve işlemci hiçbir zaman boşta kalmaz, sürekli olarak gelen veriyi ayrıştırır ve anlamlı hale getirir.


// Çift GPS için global nesnelerimizi oluşturuyoruz
// main.c içine extern etmemiz lazım kodun döndüğü yer orası olduğu için bilgilendirmemiz gerekir.
NEO_M8N_Handle_t gps_1;
NEO_M8N_Handle_t gps_2;

// GPS'i başlatır ve ilk DMA dinlemesini açar
void NEO_M8N_Init(NEO_M8N_Handle_t *hgps, UART_HandleTypeDef *huart) {
    hgps->huart = huart; // Hangi UART'ı kullanacağını belirle ( hangi GPS'te denilebilir )
    hgps->state = WAIT_SYNC1; // Cihazdan gelen ilk veriyi vermemiz lazım (0xB5). 
    hgps->is_fixed = 0; // Daha Init fonksiyonu bu yüzden struct yapımızdan is_fixed yani bağlantı var mı değişkeni bağlantı yok şeklinde kalır. Eğer var şeklinde olur ise filtrelere veya çıkışa yanlış veri basar başta
    
    // CPU'yu uyutarak USART DMA Idle kesmesini başlatıyoruz
    // Yani veri alma işlerini CPU'dan alıyoruz ve DMA'ya yaptırmaya başlıyoruz. 
    HAL_UARTEx_ReceiveToIdle_DMA(hgps->huart, hgps->rx_buffer, sizeof(hgps->rx_buffer));
}

// Checksum (Sağlamlık) hesaplama fonksiyonu (Fletcher Algorithm)
// Datasheet üzerinden nasıl hesaplandığını görebilirsiniz. Gelen her byte'ı işleyerek iki adet checksum byte'ı hesaplarız ve bunları gelen checksum byte'larıyla karşılaştırırız. Eğer aynıysa veri doğru gelmiştir, değilse veri hatalı gelmiştir ve çöpe atılır.
static void calcChecksum(uint8_t *ck_a, uint8_t *ck_b, uint8_t data) {
    *ck_a = *ck_a + data;
    *ck_b = *ck_b + *ck_a;
    // NOT: normalde direkt bu olmaz mıydı ---> *ck_a += data ---> olmazdı çünkü eğer veri gürültülü gelirse ve iki byte yeri değişirse ( 0x01, 0x02 yerine 0x02, 0x01 olursa) işlemci toplamayı aynı bulur ama veri farklıdır. Fakat ck_b sıra sıra toplama yapar ve RAM'de farklı adreslerde olduğu için veri değişimi algılanır.
    // Aslında kimse algılamaz daha kolay anlaşılabilmesi için bu şekilde yazdım aslında olasılık belkide milyarda bire düşer.
    // Olasılığın düşmesinin sebebi gürültü her zaman olabilir veya olması daha muhtemedir fakat RAM'de verinin aynı adrese yazılıp CPU'yu kanması çok daha düşük bir olasılıktır.
}

// DMA Idle kesmesi geldiğinde (Yani GPS cümeleyi bitirdiğinde) bu fonksiyon çağrılır
void NEO_M8N_ParseBuffer(NEO_M8N_Handle_t *hgps, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        uint8_t c = hgps->rx_buffer[i];
        
        switch (hgps->state) {
            case WAIT_SYNC1:
                if (c == UBX_SYNC_CHAR_1) hgps->state = WAIT_SYNC2;
                break;
            case WAIT_SYNC2:
                if (c == UBX_SYNC_CHAR_2) {
                    hgps->state = WAIT_CLASS;
                    hgps->ck_a = 0; hgps->ck_b = 0; // Checksum sıfırla
                } else hgps->state = WAIT_SYNC1;
                break;
            case WAIT_CLASS:
                hgps->class_id = c;
                calcChecksum(&hgps->ck_a, &hgps->ck_b, c);
                hgps->state = WAIT_ID;
                break;
            case WAIT_ID:
                hgps->msg_id = c;
                calcChecksum(&hgps->ck_a, &hgps->ck_b, c);
                hgps->state = WAIT_LEN1;
                break;
            case WAIT_LEN1:
                hgps->payload_len = c;
                calcChecksum(&hgps->ck_a, &hgps->ck_b, c);
                hgps->state = WAIT_LEN2;
                break;
            case WAIT_LEN2:
                hgps->payload_len |= (uint16_t)(c << 8);
                calcChecksum(&hgps->ck_a, &hgps->ck_b, c);
                hgps->payload_counter = 0;
                
                // Güvenlik: Struct boyutunu aşan saçma bir uzunluk gelirse iptal et
                if(hgps->payload_len > sizeof(UBX_NAV_PVT_t)) hgps->state = WAIT_SYNC1;
                else hgps->state = GET_PAYLOAD;
                break;
            case GET_PAYLOAD:
                // Gelen byte'ları direkt struct'ın içine yazdırıyoruz (Sihir burada!)
                ((uint8_t*)&hgps->data)[hgps->payload_counter++] = c;
                calcChecksum(&hgps->ck_a, &hgps->ck_b, c);
                
                if (hgps->payload_counter >= hgps->payload_len) hgps->state = GET_CK_A;
                break;
            case GET_CK_A:
                if (c == hgps->ck_a) hgps->state = GET_CK_B;
                else hgps->state = WAIT_SYNC1; // Hatalı paket, çöpe at
                break;
            case GET_CK_B:
                if (c == hgps->ck_b) {
                    // --- PAKET BAŞARIYLA ÇÖZÜLDÜ! ---
                    // Eğer gelen paket NAV-PVT mesajıysa Fix durumunu güncelle
                    if(hgps->class_id == UBX_NAV_CLASS && hgps->msg_id == UBX_NAV_PVT_ID) {
                        if(hgps->data.fixType == 3) hgps->is_fixed = 1; // 3D Fix var
                        else hgps->is_fixed = 0;
                        
                        // İLERİDE: Burada FreeRTOS Queue'ya "Yeni GPS datası geldi" sinyali basacağız.
                    }
                }
                hgps->state = WAIT_SYNC1; // Yeni paketi beklemeye başla
                break;
        }
    }
}