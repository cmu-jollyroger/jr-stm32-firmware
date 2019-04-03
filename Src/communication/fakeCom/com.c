//
// Created by FionaLee on 4/3/19.
//

#include "../../../Inc/MeEncoderNew.h"

typedef struct {
uint8_t index;
uint8_t speed;
} motor_cmd;

int MOTOR_NUM = 4;

int send_main(void) {
  while (1) {
    float motor_speed[MOTOR_NUM];
    motor_speed[0] = getCurrentSpeed(0);
    motor_speed[1] = getCurrentSpeed(1);
    motor_speed[2] = getCurrentSpeed(2);
    motor_speed[3] = getCurrentSpeed(3);
    write_uart_blocking(&COMPUTER_HUART, (uint8_t*)motor_speed, MOTOR_NUM * sizeof(float));
    HAL_Delay(1000);
  }
}



int recv_main(void) {
  motor_cmd cmd;
  while (1) {
    while (fgets(computer_rx_buf, UART_BUFF_SIZE, tty_file)) {
      // printf("in read: %s\n", computer_rx_buf);
      sscanf(computer_rx_buf, "%d,%d\r", &cmd.index, &cmd.speed);
      runSpeed(cmd.speed, 1, cmd.index);
      HAL_Delay(1000);
    }
}