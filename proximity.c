#include "proximity.h"
#include <stdio.h>

static bool last_s1_state = false;
static bool last_s2_state = false;
/* * ฟังก์ชันสำหรับ Initialize ขา GPIO (ใช้กรณีที่ไม่ได้ตั้งค่าผ่าน CubeMX)
 * ถ้าตั้งค่าจาก CubeMX แล้ว (Set PA8, PA9 เป็น Input Pull-Up) สามารถข้ามการเรียกฟังก์ชันนี้ได้
 */
void Proximity_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = PROXIMITY1_PIN | PROXIMITY2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // เปิดใช้งาน Pull-up ภายใน
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* * อ่านค่าจาก Sensor 1 (PA8)
 * NPN NO + Pull-up: เจอเหล็ก = 0 (LOW), ไม่เจอ = 1 (HIGH)
 * return true ถ้า "เจอเหล็ก (ตรวจจับได้)"
 */
bool Proximity_Read_Sensor1(void)
{
    bool current_state = (HAL_GPIO_ReadPin(PROXIMITY1_PORT, PROXIMITY1_PIN) == GPIO_PIN_RESET);

    // ตรวจจับจังหวะที่เซนเซอร์เปลี่ยนสถานะ
    if (current_state != last_s1_state) {
        if (current_state) {
            printf("[SENSOR 1] (PA8) -> O  DETECTED! (LOW)\r\n");
        } else {
            printf("[SENSOR 1] (PA8) -> X  Released. (HIGH)\r\n");
        }
        last_s1_state = current_state;
    }
    return current_state;
}

bool Proximity_Read_Sensor2(void)
{
    bool current_state = (HAL_GPIO_ReadPin(PROXIMITY2_PORT, PROXIMITY2_PIN) == GPIO_PIN_RESET);

    if (current_state != last_s2_state) {
        if (current_state) {
            printf("[SENSOR 2] (PA9) -> O  DETECTED! (LOW)\r\n");
        } else {
            printf("[SENSOR 2] (PA9) -> X  Released. (HIGH)\r\n");
        }
        last_s2_state = current_state;
    }
    return current_state;
}
