#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct {
  float left_mps, right_mps, clearance_m, battery_soc;
  bool valid, estop, driver_fault, arm_stowed;
} Sensors;
/* 以下函数由board.c连接STM32 HAL；反馈采集不得阻塞控制循环。 */
Sensors board_sensors(void);
void board_motor(float left_pwm, float right_pwm);
void board_safe_stop(void);
bool board_arm_start(unsigned preset); /* 必须检查限位、闭链约束和碰撞 */
void control_init(uint32_t now);
void control_step(uint32_t now); /* 主循环每10ms调用；不得在阻塞中断中调用 */
void control_command(uint8_t type, float a, float b, uint32_t now);
/* UART接收完整16字节帧后，在主循环调用。 */
bool control_frame(const uint8_t frame[16], uint32_t now);
