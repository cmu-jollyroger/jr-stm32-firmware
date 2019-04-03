//
// Created by FionaLee on 4/3/19.
//

/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file comm_task.c
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief communicate with computer task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "comm_task.h"
#include "protocol.h"
#include "communicate.h"
#include "infantry_info.h"
#include "info_interactive.h"
#include "sys_config.h"
#include "bsp_uart.h"
#include "pid.h"
#include "cmsis_os.h"
#include "string.h"
#include "../../Inc/MeEncoderNew.h"
#include "../../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

UBaseType_t freq_info_surplus;
UBaseType_t pc_comm_surplus;
UBaseType_t judge_comm_surplus;
UBaseType_t can_send_surplus;

/* communicate task global parameter */
motor_current_t glb_cur;

/* pc receive data fifo and buffer */
static osMutexId pc_rxdata_mutex;
fifo_s_t  pc_rxdata_fifo;
uint8_t   pc_rxdata_buf[COMPUTER_FIFO_BUFLEN];
/* pc send data fifo and buffer */
static osMutexId pc_txdata_mutex;
static fifo_s_t  pc_txdata_fifo;
static uint8_t   pc_txdata_buf[COMPUTER_FIFO_BUFLEN];
/* pc system dma receive data object */
static uart_dma_rxdata_t pc_rx_obj;

/* unpack object */
static unpack_data_t pc_unpack_obj;

int MOTOR_ID  = 0;
int MOTOR_NUM = 4;

//uint32_t can_time_last;
//int can_time_ms;
//uint32_t can1_time_last;
//int can1_time_ms;
void can_msg_send_task(void const *argu)
{
  osEvent event;

  while (1)
  {
    event = osSignalWait(CHASSIS_MOTOR_MSG_SEND, osWaitForever);

    if (event.status == osEventSignal)
    {
      // always true given no gimbal
      if (1 || event.value.signals & CHASSIS_MOTOR_MSG_SEND)
      {
//        can1_time_ms = HAL_GetTick() - can1_time_last;
//        can1_time_last = HAL_GetTick();
        send_chassis_motor_ctrl_message(glb_cur.chassis_cur);
      }
    }
    can_send_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

uint32_t comm_time_last;
int comm_time_ms;

uint8_t tx_packet_id = CHASSIS_DATA_ID;
int count_cali_count = 0;
extern TaskHandle_t pc_unpack_task_t;
void freq_info_task(void const *argu)
{
  uint32_t comm_wake_time = osKernelSysTick();

  while (1)
  {
    comm_time_ms = HAL_GetTick() - comm_time_last;
    comm_time_last = HAL_GetTick();

    get_upload_data();


    if (count_cali_count == 1)   //50Hz
    {
      count_cali_count = 0;

      float motor_speed[MOTOR_NUM];
      motor_speed[0] = getCurrentSpeed(0);
      motor_speed[1] = getCurrentSpeed(1);
      motor_speed[2] = getCurrentSpeed(2);
      motor_speed[3] = getCurrentSpeed(3);

      data_packet_pack(MOTOR_ID, (uint8_t *)motor_speed, MOTOR_NUM * sizeof(float), UP_REG_ID);

//      switch (tx_packet_id)
//      {
//        case CHASSIS_DATA_ID:
//        {
//          /* Haowen: This will send chassis information to PC */
//          data_packet_pack(CHASSIS_DATA_ID, (uint8_t *)&pc_send_mesg.chassis_information,
//                           sizeof(chassis_info_t), UP_REG_ID);
//        }break;

//        case INFANTRY_ERR_ID:
//        {
//          data_packet_pack(INFANTRY_ERR_ID, (uint8_t *)&pc_send_mesg.bottom_error_data,
//                           sizeof(infantry_err_t), UP_REG_ID);
//          tx_packet_id = CONFIG_RESPONSE_ID;
//        }break;

//        case CONFIG_RESPONSE_ID:
//        {
//          data_packet_pack(CONFIG_RESPONSE_ID, (uint8_t *)&pc_send_mesg.structure_config_data,
//                           sizeof(config_response_t), UP_REG_ID);
//          tx_packet_id = REMOTE_CTRL_INFO_ID;
//        }break;

//        case REMOTE_CTRL_INFO_ID:
//        {
//          data_packet_pack(REMOTE_CTRL_INFO_ID, (uint8_t *)&pc_send_mesg.remote_ctrl_data,
//                           sizeof(rc_info_t), UP_REG_ID);
//          tx_packet_id = BOTTOM_VERSION_ID;
//        }break;

//        case BOTTOM_VERSION_ID:
//        {
//          data_packet_pack(BOTTOM_VERSION_ID, (uint8_t *)&pc_send_mesg.version_info_data,
//                           sizeof(version_info_t), UP_REG_ID);
//          tx_packet_id = CHASSIS_DATA_ID;
//        }break;
      }
    }
    else  //50Hz
    {
      count_cali_count++;
      // do something else here at 50Hz
    }

    osSignalSet(pc_unpack_task_t, PC_UART_TX_SIGNAL);

    freq_info_surplus = uxTaskGetStackHighWaterMark(NULL);

    osDelayUntil(&comm_wake_time, COMM_TASK_PERIOD);  //100Hz
  }

}

extern TaskHandle_t freq_info_task_t;
//uint8_t pc_rx_test[40];
void pc_unpack_task(void const *argu)
{
  osEvent event;

  taskENTER_CRITICAL();
  /* open pc uart receive it */
  computer_uart_init();
  /* create periodic information task */
  osThreadDef(commTask, freq_info_task, osPriorityNormal, 0, 256);
  freq_info_task_t = osThreadCreate(osThread(commTask), NULL);
  taskEXIT_CRITICAL();

  while (1)
  {
    event = osSignalWait(PC_UART_TX_SIGNAL | \
                         PC_UART_IDLE_SIGNAL, osWaitForever);
    /* Haowen: upon IDLE, will receive data from PC */
    if (event.status == osEventSignal)
    {
      //receive pc data puts fifo
      if (event.value.signals & PC_UART_IDLE_SIGNAL)
      {
        dma_buffer_to_unpack_buffer(&pc_rx_obj, UART_IDLE_IT);
        unpack_fifo_data(&pc_unpack_obj, UP_REG_ID);
      }

      //send data to pc system
      if (event.value.signals & PC_UART_TX_SIGNAL)
      {
        send_packed_fifo_data(&pc_txdata_fifo, UP_REG_ID);
      }

    }

    pc_comm_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

void get_upload_data(void)
{
  taskENTER_CRITICAL();
  //taskENTER_CRITICAL_FROM_ISR()

  get_infantry_info();
  get_custom_data_info();

  taskEXIT_CRITICAL();
  //taskEXIT_CRITICAL_FROM_ISR
}


void communicate_param_init(void)
{

  /* create the pc_rxdata_mutex mutex  */
  osMutexDef(pc_rxdata_mutex);
  pc_rxdata_mutex = osMutexCreate(osMutex(pc_rxdata_mutex));

  /* create the pc_txdata_mutex mutex  */
  osMutexDef(pc_txdata_mutex);
  pc_txdata_mutex = osMutexCreate(osMutex(pc_txdata_mutex));

  /* pc data fifo init */
  fifo_s_init(&pc_rxdata_fifo, pc_rxdata_buf, COMPUTER_FIFO_BUFLEN, pc_rxdata_mutex);
  fifo_s_init(&pc_txdata_fifo, pc_txdata_buf, COMPUTER_FIFO_BUFLEN, pc_txdata_mutex);

  /* initial pc data dma receiver object */
  pc_rx_obj.huart = &COMPUTER_HUART;
  pc_rx_obj.data_fifo = &pc_rxdata_fifo;
  pc_rx_obj.buff_size = UART_RX_DMA_SIZE;
  pc_rx_obj.buff[0] = pc_dma_rxbuff[0];
  pc_rx_obj.buff[1] = pc_dma_rxbuff[1];

  /* initial pc data unpack object */
  pc_unpack_obj.data_fifo = &pc_rxdata_fifo;
  pc_unpack_obj.p_header = (frame_header_t *)pc_unpack_obj.protocol_packet;
  pc_unpack_obj.index = 0;
  pc_unpack_obj.data_len = 0;
  pc_unpack_obj.unpack_step = STEP_HEADER_SOF;

  memset(&glb_cur, 0, sizeof(motor_current_t));

  pc_send_mesg.version_info_data.num[0] = 2;
  pc_send_mesg.version_info_data.num[1] = 0;
  pc_send_mesg.version_info_data.num[2] = 1;
  pc_send_mesg.version_info_data.num[3] = 8;

}

void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof)
{
  uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];

  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;

  protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);

  /* use mutex operation */
  if (sof == UP_REG_ID)
    fifo_s_puts(&pc_txdata_fifo, tx_buf, frame_length);
  else
    return ;
}