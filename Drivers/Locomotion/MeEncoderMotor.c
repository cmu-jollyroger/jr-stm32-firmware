#include <stdio.h>
#include <stdint.h>

#include "MeEncoderMotor.h"
#include "i2c_shared.h"
#include "stm32xxx_hal.h"

/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class   MeEncoderMotor
 * \brief   Driver for Encoder Motor module.
 * @file    MeEncoderMotor.cpp
 * @author  MakeBlock
 * @version V1.0.1
 * @date    2015/11/09
 * @brief   Driver for Encoder Motor module
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is a drive for Me EncoderMotor device, The Me EncoderMotor inherited the
 * MeSerial class from SoftwareSerial.
 *
 * \par Method List:
 *
 *    1. void begin();
 *    2. bool reset();
 *    3. bool move(float angle, float speed);
 *    4. bool moveTo(float angle, float speed);
 *    5. bool runTurns(float turns, float speed);
 *    6. bool runSpeed(float speed);
 *    7. bool runSpeedAndTime(float speed, float time);
 *    8. float getCurrentSpeed();
 *    9. float getCurrentPosition();
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan        2015/11/03     1.0.0            Fix minor errors on format
 * forfish         2015/11/09     1.0.1            Add description
 * </pre>
 *
 * @example EncoderMotorTestMoveTo.ino
 * @example EncoderMotorTestRunSpeed.ino
 * @example EncoderMotorTestRunSpeedAndTime.ino
 * @example EncoderMotorTestRunTurns.ino
 */
#include "MeEncoderMotor.h"
#include "MeHostParser.h"

#define ENCODER_MOTOR_GET_PARAM     0x01
#define ENCODER_MOTOR_SAVE_PARAM    0x02
#define ENCODER_MOTOR_TEST_PARAM    0x03
#define ENCODER_MOTOR_SHOW_PARAM    0x04
#define ENCODER_MOTOR_RUN_STOP      0x05
#define ENCODER_MOTOR_GET_DIFF_POS  0x06
#define ENCODER_MOTOR_RESET         0x07
#define ENCODER_MOTOR_SPEED_TIME    0x08
#define ENCODER_MOTOR_GET_SPEED     0x09
#define ENCODER_MOTOR_GET_POS       0x10
#define ENCODER_MOTOR_MOVE          0x11
#define ENCODER_MOTOR_MOVE_TO       0x12
#define ENCODER_MOTOR_DEBUG_STR     0xCC
#define ENCODER_MOTOR_ACKNOWLEDGE   0xFF


typedef struct meEncoder {
uint8_t _slot;
uint8_t _slaveAddress;
unsigned long _lastTime;
} MeEncoder;

#define NumOfEncoder 4
MeEncoder MeEncoders[NumOfEncoder];
MeParser meEncoderParser;
MeParser* encoderParser = &meEncoderParser;

/**
 * \par Function
 *    MeHost_Pack
 * \par Description
 *    Pack data into a package to send.
 * \param[in]
 *    buf - Buffer to save package.
 * \param[in]
 *    bufSize - Size of buf.
 * \param[in]
 *    module - The associated module of package.
  * \param[in]
 *    data - The data to pack.
 * \param[in]
 *    length - The length(size) of data.
 * \par Output
 *    None
 * \Return
 *    0.
 * \par Others
 *    package size.
 */
uint32_t MeHost_Pack(uint8_t * buf,
                     uint32_t bufSize,
                     uint8_t module,
                     uint8_t * data,
                     uint32_t length)
{
  uint32_t i = 0;

  //  head: 0xA5
  buf[i++] = 0xA5;
  buf[i++] = module;
  //  pack length
  buf[i++] = *((uint8_t *)&length + 0);
  buf[i++] = *((uint8_t *)&length + 1);
  buf[i++] = *((uint8_t *)&length + 2);
  buf[i++] = *((uint8_t *)&length + 3);
  //  pack data
  for(uint32_t j = 0; j < length; ++j)
  {
    buf[i++] = data[j];
  }

  //  calculate the LRC
  uint8_t check = 0x00;
  for(uint32_t j = 0; j < length; ++j)
  {
    check ^= data[j];
  }
  buf[i++] = check;

  //  tail: 0x5A
  buf[i++] = 0x5A;

  if (i > bufSize)
  {
    return 0;
  }
  else
  {
    return i;
  }
}

/**
 * Alternate Constructor which can call your own function to map the Encoder Motor to arduino port,
 * you can set any slot for the Encoder Motor device.
 * \param[in]
 *   port - RJ25 port from PORT_1 to M2
 * \param[in]
 *   slot - SLOT1 or SLOT2
 */
void MeEncoderMotorAddrSlot(uint8_t addr, uint8_t slot, int idx)
{
  MeEncoders[idx]._slot = (uint8_t) (slot - 1);
  MeEncoders[idx]._slaveAddress = addr;
}

/**
 * Alternate Constructor which can call your own function to map the Encoder Motor to arduino port,
 * you can set any slot for the Encoder Motor device.
 * \param[in]
 *   slot - SLOT1 or SLOT2
 */
void MeEncoderMotorSlot(uint8_t slot, int idx)
{
  MeEncoders[idx]._slot = slot - 1;
  MeEncoders[idx]._slaveAddress = 0x9;
}

/**
 * Alternate Constructor which can call your own function to map the Encoder Motor to arduino port,
 * you should initialized slot and slaveAddress here for the Encoder Motor device.
 * \param[in]
 *   None
 */
void MeEncoderMotor(int idx)
{
  MeEncoders[idx]._slot = 0;
  MeEncoders[idx]._slaveAddress = 0x9;
}

/**
 * \par Function
 *    begin
 * \par Description
 *    Initialize Encoder Motor.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void begin(int idx)
{
  // TODO: setup sda scl pins
  reset(idx);
}

/**
 * \par Function
 *   reset
 * \par Description
 *   Reset the available data for Encoder Motor.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
bool reset(int idx)
{
  uint8_t w[10] = {0};
  uint8_t r[10] = {0};

  uint8_t data[2] = {0};
  data[0] = MeEncoders[idx]._slot;
  data[1] = ENCODER_MOTOR_RESET;

  MeHost_Pack(w, 10, 0x01, data, 2);
  request(w, r, 10, 10, idx);
  pushStr(encoderParser, r, 10);

  uint8_t ack[2] = {0};
  getData(encoderParser, ack, 2);
  return ack[1];
}

/**
 * \par Function
 *    moveTo
 * \par Description
 *    Motor move to the aim.
 * \param[in]
 *    angle - The angle move of Motor.
 * \param[in]
 *    speed - The speed move of Motor.
 * \par Output
 *    None
 * \par Return
 *    Return the result of Motor's movement to the aim.
 * \par Others
 *    None
 */
bool moveTo(float angle, float speed, int idx)
{
  if(speed > 200)
  {
    speed = 200;
  }
  else if(speed < -200)
  {
    speed = -200;
  }
  uint8_t w[18] = {0};
  uint8_t r[10] = {0};

  uint8_t data[10] = {0};
  data[0] = MeEncoders[idx]._slot;
  data[1] = ENCODER_MOTOR_MOVE_TO;
  *((float *)(data + 2)) = angle;
  *((float *)(data + 6)) = speed;

  MeHost_Pack(w, 18, 0x01, data, 10);
  request(w, r, 18, 10, idx);
  pushStr(encoderParser, r, 10);
  run(encoderParser);

  uint8_t ack[2] = {0};
  getData(encoderParser, ack, 2);
  return ack[1];
}

/**
 * \par Function
 *    move
 * \par Description
 *    Motor move.
 * \param[in]
 *    angle - The angle move of Motor.
 * \param[in]
 *    speed - The speed move of Motor.
 * \par Output
 *    None
 * \par Return
 *    Return the result of Motor's movement.
 * \par Others
 *    None
 */
bool move(float angle, float speed, int idx)
{
  if(speed > 200)
  {
    speed = 200;
  }
  else if(speed < -200)
  {
    speed = -200;
  }
  if(angle == 0)
  {
    return runSpeed(speed, idx);
  }
  uint8_t w[18] = {0};
  uint8_t r[10] = {0};

  uint8_t data[10] = {0};
  data[0] = MeEncoders[idx]._slot;
  data[1] = ENCODER_MOTOR_MOVE;
  *((float *)(data + 2)) = angle;
  *((float *)(data + 6)) = speed;

  MeHost_Pack(w, 18, 0x01, data, 10);
  request(w, r, 18, 10, idx);
  pushStr(encoderParser, r, 10);
  run(encoderParser);
  uint8_t ack[2] = {0};
  getData(encoderParser, ack, 2);
  return ack[1];
}

/**
 * \par Function
 *    runTurns
 * \par Description
 *    Motor move turns.
 * \param[in]
 *    turns - The turns move of Motor.
 * \param[in]
 *    speed - The speed move of Motor.
 * \par Output
 *    None
 * \par Return
 *    Return the result of Motor's movement.
 * \par Others
 *    None
 */
bool runTurns(float turns, float speed, int idx)
{
  return move(turns * 360, speed, idx);
}

/**
 * \par Function
 *    runSpeed
 * \par Description
 *    The speed of Motor's movement.
 * \param[in]
 *    speed - The speed move of Motor.
 * \par Output
 *    None
 * \par Return
 *    Return 0.
 * \par Others
 *    None
 */
bool runSpeed(float speed, int idx)
{
  if(speed > 200)
  {
    speed = 200;
  }
  else if(speed < -200)
  {
    speed = -200;
  }
  uint8_t w[14] = {0};
  uint8_t r[10] = {0};

  uint8_t data[12] = {0}; // 6, extra space for stack padding
  data[0 + 6] = MeEncoders[idx]._slot;
  data[1 + 6] = ENCODER_MOTOR_RUN_STOP;
  *((float *)(data + 2 + 6)) = speed;

  MeHost_Pack(w, 14, 0x01, data + 6, 6);
  request(w, r, 14, 10, idx);
  pushStr(encoderParser, r, 10);
  run(encoderParser);

  // uint8_t ack[2] = {0};
  // encoderParser.GetData(ack, 2);
  // return ack[1];
  return 0;
}

/**
 * \par Function
 *    runSpeedAndTime
 * \par Description
 *    The speed and time of Motor's movement.
 * \param[in]
 *    speed - The speed move of Motor.
 * \param[in]
 *    time - The time move of Motor.
 * \par Output
 *    None
 * \par Return
 *    Return the result of Motor's movement.
 * \par Others
 *    None
 */
bool runSpeedAndTime(float speed, float time, int idx)
{
  if(speed > 200)
  {
    speed = 200;
  }
  else if(speed < -200)
  {
    speed = -200;
  }
  uint8_t w[18] = {0};
  uint8_t r[10] = {0};

  uint8_t data[10] = {0};
  data[0] = MeEncoders[idx]._slot;
  data[1] = ENCODER_MOTOR_SPEED_TIME;
  *((float *)(data + 2)) = speed;
  *((float *)(data + 6)) = time;

  MeHost_Pack(w, 18, 0x01, data, 10);
  request(w, r, 18, 10, idx);
  pushStr(encoderParser, r, 10);
  run(encoderParser);

  return 0;
}

/**
 * \par Function
 *    getCurrentSpeed
 * \par Description
 *    The current speed of Motor's movement.
 * \param[in]
      None
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
float getCurrentSpeed(int idx)
{
  uint8_t w[10] = {0};
  uint8_t r[14] = {0};

  uint8_t data[2] = {0};
  data[0] = MeEncoders[idx]._slot;
  data[1] = ENCODER_MOTOR_GET_SPEED;

  MeHost_Pack(w, 10, 0x01, data, 2);
  request(w, r, 10, 14, idx);
  pushStr(encoderParser, r, 14);
  run(encoderParser);

  uint8_t temp[6] = {0};
  getData(encoderParser, temp, 6);
  float speed = *((float *)(temp + 2));
  return speed;
}

/**
 * \par Function
 *    getCurrentPosition
 * \par Description
 *    The current position of Motor.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
float getCurrentPosition(int idx)
{
  uint8_t w[10] = {0};
  uint8_t r[14] = {0};

  uint8_t data[2] = {0};
  data[0] = MeEncoders[idx]._slot;
  data[1] = ENCODER_MOTOR_GET_POS;

  MeHost_Pack(w, 10, 0x01, data, 2);
  request(w, r, 10, 14, idx);
  pushStr(encoderParser, r, 14);

  run(encoderParser);

  uint8_t temp[6] = {0};
  uint8_t size = getData(encoderParser, temp, 6);
  float pos = *((float *)(temp + 2));
  return pos;
}

/**
 * \par Function
 *    request
 * \par Description
 *    The request of Motor.
 * \param[in]
 *    writeData - Write data to Encoder Motor.
  * \param[in]
 *    readData - Read data from Encoder Motor.
  * \param[in]
 *    wlen - The data's length that write to Encoder Motor.
  * \param[in]
 *    rlen - The data's length that read from Encoder Motor.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void request(uint8_t *writeData, uint8_t *readData, int wlen, int rlen, int idx)
{
	printf("requesting data\r\n");
  //uint8_t rxByte;
  //uint8_t index = 0;
  uint32_t timeout = 500;

  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Master_Transmit(&hi2c1, (MeEncoders[idx]._slaveAddress) << 1, writeData, wlen, timeout);
  if (status == HAL_OK) {
    HAL_Delay(2);
		//HAL_I2C_Master_Receive(Dev->I2cHandle, Dev->I2cDevAddr|1, pdata, count, i2c_time_out);
    status = HAL_I2C_Master_Receive(&hi2c1, ((MeEncoders[idx]._slaveAddress) << 1) | 1, readData, rlen, timeout);
  }
	if (status == HAL_OK) {
		return;
	}

//  Wire.beginTransmission(MeEncoders[idx]._slaveAddress); // transmit to device

//  Wire.write(writeData, wlen);

//  Wire.endTransmission();
//
//  delayMicroseconds(2);
//  Wire.requestFrom((int)MeEncoders[idx]._slaveAddress, (int)rlen); // request 6 bytes from slave device
//  delayMicroseconds(2);
//
//  while(Wire.available()) // slave may send less than requested
//  {
//    rxByte = Wire.read(); // receive a byte as character
//    readData[index] = rxByte;
//    index++;
//  }
}

void MeEncoderDriver_Config() {
	return;
}

void MeEncoderDriver_Init() {
	// Initialize two motors
	MeEncoderMotorSlot(SLOT1, 0);
	MeEncoderMotorSlot(SLOT2, 1);
	
	MeHostParserInit(encoderParser);
	
	begin(0);
	begin(1);
}
