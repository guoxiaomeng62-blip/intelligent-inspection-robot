#pragma once
#include "stm32f4xx_hal.h"
#include <stdbool.h>
typedef struct {
 TIM_HandleTypeDef *timer;
 uint32_t channel;
 GPIO_TypeDef *dir_port;
 uint16_t dir_pin;
 int direction, pending;
 uint32_t changed_at;
} Motor;
void motor_write(Motor *m, float pwm, uint32_t now);
bool pca_init(I2C_HandleTypeDef *bus);
bool pca_servo(I2C_HandleTypeDef *bus, unsigned channel, unsigned pulse_us);
/* 每个HC-SR04独立一个对象；使用1MHz自由运行TIM5时间戳。 */
typedef struct { volatile uint32_t rise, stamp, width; volatile unsigned phase;
 uint32_t started; } Sonar;
bool sonar_trigger(Sonar *s, GPIO_TypeDef *port, uint16_t pin,
                   TIM_HandleTypeDef *us_timer, uint32_t now);
void sonar_edge(Sonar *s, bool high, uint32_t micros, uint32_t now);
bool sonar_read(Sonar *s, uint32_t now, float *metres);
float encoder_mps(uint16_t count, uint16_t *previous, float metres_per_count, float dt);
