#ifndef INC_CAN_MANAGER_H_
#define INC_CAN_MANAGER_H_

#include "stm32f4xx_hal.h"

// --- NEDEN? ---
// Hattaki diğer cihazlar (Raspberry Pi) gelen verinin GPS mi, İvme mi olduğunu
// buradaki ID'ye bakarak anlayacak. 11-bitlik (Standart) ID kullanıyoruz.

#define CAN_ID_CRITICAL_ERROR   0x050  // En yüksek öncelik! (Sensör koptu vs.)
#define CAN_ID_IMU_ACCEL        0x100  // İvme (Otonom sürüş için kritik)
#define CAN_ID_IMU_GYRO         0x101  // Gyro
#define CAN_ID_GPS_POS          0x110  // Enlem/Boylam
#define CAN_ID_GPS_SPEED        0x111  // Hız/Heading
#define CAN_ID_SYSTEM_STATUS    0x150  // Kalp atışı (Düşük öncelik)
#define CAN_ID_CMD_TO_SENSOR    0x0A0  // Jetsonun bu karta emir vermesi için kullanıdğı ID

// Fonksiyonlarımızı dünyaya duyuruyoruz (hcan parametresi eklendi!)
void CAN_Config_Init(CAN_HandleTypeDef *hcan);
uint8_t CAN_Send_Message(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len);

#endif /* INC_CAN_MANAGER_H_ */