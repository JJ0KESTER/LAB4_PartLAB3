#ifndef PROXIMITY_H
#define PROXIMITY_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>

/* ── กำหนดขา GPIO สำหรับ Proximity Sensor ── */
#define PROXIMITY1_PIN  GPIO_PIN_8
#define PROXIMITY1_PORT GPIOA

#define PROXIMITY2_PIN  GPIO_PIN_9
#define PROXIMITY2_PORT GPIOA

/* ── ฟังก์ชันการใช้งาน ── */
void Proximity_Init(void);
bool Proximity_Read_Sensor1(void);
bool Proximity_Read_Sensor2(void);

#endif /* PROXIMITY_H */
