/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class   MeEncoderNew
 * \brief   Driver for Me Encoder New module.
 * @file    MeEncoderNew.cpp
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/03/18
 * @brief   Driver for Me Encoder New module.
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
 * This file is a drive for Me EncoderNew device, The Me EncoderNew inherited the
 * MeSerial class from SoftwareSerial.
 *
 * \par Method List:
 *
 *    1. void begin(void);
 *    2. void setAddr(uint8_t i2cAddr,uint8_t slot);
 *    3. void move(long angle, float speed, float lock_state);
 *    4. void moveTo(long angle, float speed,float lock_state);
 *    5. void runSpeed(int speed);
 *    6. void runTurns(long turns, float speed,float lock_state);
 *    7. void reset(void);
 *    8. void setSpeedPID(float p,float i,float d);
 *    9. void setPosPID(float p,float i,float d);
 *    10. void setMode(uint8_t mode);
 *    11. void setPWM(int pwm);
 *    12. void setCurrentPosition(long pulse_counter)
 *    13. long getCurrentPosition();
 *    14. void getSpeedPID(float * p,float * i,float * d);
 *    15. void getPosPID(float * p,float * i,float * d);
 *    16. float getCurrentSpeed(void);
 *    17. void sendCmd(idx)(void);
 *    18. float getRatio(void);
 *    19. void setRatio(float r);
 *    20. int getPulse(void);
 *    21. void setPulse(int p);
 *    22. void setDevid(int devid);
 *    23. void runSpeedAndTime(float speed, float time, float lock_state);
 *    24. bool isTarPosReached(void);
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan        2016/03/18     1.0.0            build the new
 * Mark Yan        2017/06/09     1.0.1            add function setAddr
 * </pre>
 *
 * @example EncoderMotorChangeI2CDevID.ino
 * @example EncoderMotorTestIsTarPosReached.ino
 * @example EncoderMotorTestMoveTo.ino
 * @example EncoderMotorTestRunSpeed.ino
 * @example EncoderMotorTestRunSpeedAndTime.ino
 * @example EncoderMotorTestRunTurns.ino
 */

#include <string.h>
#include "MeEncoderNew.h"

#include "i2c_shared.h"
#include "stm32xxx_hal.h"

#define HOLD 0x40
#define SYNC 0x80
// move state and function
#define CMD_RESET         0x00
#define CMD_MOVE_TO       0x01
#define CMD_MOVE          0x02
#define CMD_MOVE_SPD      0x03
#define CMD_STOP          0x05

// config function
#define CMD_SET_SPEED_PID 0x10
#define CMD_SET_POS_PID   0x11
#define CMD_SET_CUR_POS   0x12
#define CMD_SET_MODE      0x13
#define CMD_SET_PWM       0x14
#define CMD_SET_RATIO     0x15
#define CMD_SET_PULSE     0x16
#define CMD_SET_DEVID     0x17
// get motor status
#define CMD_GET_SPEED_PID       0x20
#define CMD_GET_POS_PID         0x21
#define CMD_GET_POS             0x23
#define CMD_GET_SPEED           0x24
#define CMD_GET_RATIO           0x25
#define CMD_GET_PULSE           0x26
#define CMD_GET_LOCK_STATE      0x27
#define CMD_GET_FIRWARE_VERSION 0x30

#define Timeout 500
#define CmdBufLength 18
typedef struct meEncoder {
  uint8_t _slot;
  uint8_t address;
  unsigned long _lastTime;
  uint8_t cmdBuf[CmdBufLength];

} MeEncoder;

#define NumOfEncoder 4
MeEncoder MeEncoders[NumOfEncoder];


void I2C_write(uint8_t *writeData, int wlen, int idx, uint32_t timeout);
void I2C_read( uint8_t *readData, int rlen, int idx, uint32_t timeout);

/**
 * Alternate Constructor which can call your own function to map the Encoder Motor New to arduino port,
 * you can set any slot for the Encoder Motor New device.
 * \param[in]
 *   port - RJ25 port from PORT_1 to PORT_10
 * \param[in]
 *   slot - SLOT1 or SLOT2
 */
void MeEncoderNewAddrSlot(uint8_t addr, uint8_t slot, int idx) {
  MeEncoders[idx]._slot = slot - 1;
  MeEncoders[idx].address = addr;
}

/**
 * Alternate Constructor which can call your own function to map the Encoder Motor New to arduino port,
 * you can set any slot for the Encoder Motor New device.
 * \param[in]
 *   slot - SLOT1 or SLOT2
 */
void MeEncoderNewSlot(uint8_t slot, int idx)
{
  MeEncoders[idx]._slot = slot - 1;
  MeEncoders[idx].address = 0x9;
}

/**
 * Alternate Constructor which can call your own function to map the Encoder Motor New to arduino port,
 * you should initialized slot and slaveAddress here for the Encoder Motor New device.
 * \param[in]
 *   None
 */
void MeEncoderNew(int idx)
{
  MeEncoders[idx].address = 0x09;
}

/**
 * \par Function
 *    begin
 * \par Description
 *    Initialize Encoder Motor New.
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
//  Wire.begin();
}

/**
 * \par Function
 *    setAddr
 * \par Description
 *     Reset the i2c address of encoder motor .
 * \param[in]
 *    i2cAddr - i2c address of encoder motor
 * \param[in]
 *    slot - slot number of encoder motor
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setAddr(uint8_t i2cAddr,uint8_t slot, int idx)
{
  MeEncoders[idx].address = i2cAddr;
  MeEncoders[idx]._slot = slot - 1;
}

/**
 * \par Function
 *    move
 * \par Description
 *    Motor move.
 * \param[in]
 *    angle - The angle move of Motor New.
 * \param[in]
 *    speed - The speed move of Motor New.
 * \param[in]
 *    lock_state - The lock state of Motor.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void move(long angle, float speed, float lock_state, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_MOVE;
  memcpy(MeEncoders[idx].cmdBuf + 2, &lock_state, 4);
  memcpy(MeEncoders[idx].cmdBuf + 6, &angle, 4);
  memcpy(MeEncoders[idx].cmdBuf + 10,&speed,4);
  sendCmd(idx);
  HAL_Delay(2);
}

/**
 * \par Function
 *    moveTo
 * \par Description
 *    Motor New move to the aim.
 * \param[in]
 *    angle - The angle move of Motor New.
 * \param[in]
 *    speed - The speed move of Motor New.
 * \param[in]
 *    lock_state - The lock state of Motor.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void moveTo(long angle, float speed,float lock_state, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_MOVE_TO;
  memcpy(MeEncoders[idx].cmdBuf + 2, &lock_state, 4);
  memcpy(MeEncoders[idx].cmdBuf + 6, &angle, 4);
  memcpy(MeEncoders[idx].cmdBuf + 10,&speed,4);
  sendCmd(idx);
  HAL_Delay(2);
}

/**
 * \par Function
 *    runSpeed
 * \par Description
 *    The speed of Motor's movement.
 * \param[in]
 *    speed - The speed move of Motor.
 * \param[in]
 *    lock_state - The lock state of Motor.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void runSpeed(float speed,float lock_state, int idx)
{
//  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
//  MeEncoders[idx].cmdBuf[1] = CMD_MOVE_SPD;
//  memcpy(MeEncoders[idx].cmdBuf + 2, &lock_state, 4);
//  memcpy(MeEncoders[idx].cmdBuf + 6, &speed, 4);
//  sendCmd(idx);
//  HAL_Delay(2);

  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_MOVE_SPD;
  memcpy(MeEncoders[idx].cmdBuf + 2, &lock_state, 4);
  memcpy(MeEncoders[idx].cmdBuf + 6, &speed, 4);
  sendCmd(idx);
  HAL_Delay(2);
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
 * \param[in]
 *    lock_state - The lock state of Motor.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void runTurns(long turns, float speed,float lock_state, int idx)
{
  move(turns * 360, speed,lock_state, idx);
}

/**
 * \par Function
 *    reset
 * \par Description
 *    Reset the parameter of Motor.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void reset(int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_RESET;
  sendCmd(idx);
  HAL_Delay(2);
}

/**
 * \par Function
 *    setSpeedPID
 * \par Description
 *    Set speed PID for Motor.
 * \param[in]
 *    p - P means Proportion.
 * \param[in]
 *    i - I means Integration.
 * \param[in]
 *    d - D means Differentiation.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setSpeedPID(float p,float i,float d, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_SET_SPEED_PID;
  memcpy(&(MeEncoders[idx].cmdBuf[2]),&p,4);
  memcpy(&(MeEncoders[idx].cmdBuf[6]),&i,4);
  memcpy(&(MeEncoders[idx].cmdBuf[10]),&d,4);
  sendCmd(idx);
}

/**
 * \par Function
 *    setPosPID
 * \par Description
 *    Set pos PID for Motor.
 * \param[in]
 *    p - P means Proportion.
 * \param[in]
 *    i - I means Integration.
 * \param[in]
 *    d - D means Differentiation.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setPosPID(float p,float i,float d, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_SET_POS_PID;
  memcpy(&(MeEncoders[idx].cmdBuf[2]),&p,4);
  memcpy(&(MeEncoders[idx].cmdBuf[6]),&i,4);
  memcpy(&(MeEncoders[idx].cmdBuf[10]),&d,4);
  sendCmd(idx);
}

/**
 * \par Function
 *    setMode
 * \par Description
 *    Set the work mode to Motor.
 * \param[in]
 *    mode - The work mode of Motor.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setMode(uint8_t mode, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_SET_MODE;
  MeEncoders[idx].cmdBuf[2] = mode;
  sendCmd(idx);
  HAL_Delay(2);
}

/**
 * \par Function
 *    setPWM
 * \par Description
 *    Set the PWM to Motor.
 * \param[in]
 *    pwm - Pulse-Width Modulation.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setPWM(int pwm, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_SET_PWM;
  memcpy(MeEncoders[idx].cmdBuf+2,&pwm,2);
  sendCmd(idx);
  HAL_Delay(2);
}

/**
 * \par Function
 *    setCurrentPosition
 * \par Description
 *    Set current position of Motor.
 * \param[in]
 *    pulse_counter - The count value of current encoder
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setCurrentPosition(long pulse_counter, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_SET_CUR_POS;
  memcpy(MeEncoders[idx].cmdBuf+2,&pulse_counter,4);
  sendCmd(idx);
  HAL_Delay(2);
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
 *    Motor encoder pulse count value.
 * \par Others
 *    None
 */
long getCurrentPosition( int idx)
{
  long pos;
  uint8_t buf[8];
//  Wire.beginTransmission(MeEncoders[idx].address);
//  Wire.write(MeEncoders[idx]._slot);
//  Wire.write(CMD_GET_POS);
//  Wire.endTransmission(0);
//  Wire.requestFrom(MeEncoders[idx].address,(uint8_t)4);
//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }

  uint8_t request_code = CMD_GET_POS;
  uint8_t rlen = 4;
  request(&request_code, buf, rlen, idx);

  pos = *(long*)buf;
  return pos;
}

/**
 * \par Function
 *    getSpeedPID
 * \par Description
 *    Get PID from Motor.
 * \param[in]
 *    p - P means Proportion.
 * \param[in]
 *    i - I means Integration.
 * \param[in]
 *    d - D means Differentiation.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void getSpeedPID(float * p,float * i,float * d, int idx)
{
//  uint8_t buf[8];
  uint8_t buf[12];
//  Wire.beginTransmission(MeEncoders[idx].address);
//  Wire.write(MeEncoders[idx]._slot);
//  Wire.write(CMD_GET_SPEED_PID);
//  Wire.endTransmission(0);
//  Wire.requestFrom(MeEncoders[idx].address,(uint8_t)12);

  uint8_t request_code = CMD_GET_SPEED_PID;
  uint8_t rlen = 12;
  request(&request_code, buf, rlen, idx);

//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }
//  *p = *(float*)buf;
//
//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }
//  *i = *(float*)buf;
//
//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }
//  *d = *(float*)buf;
  float *PIDbuf;
  PIDbuf = (float *)buf;
  *p = PIDbuf[0];
  *i = PIDbuf[1];
  *d = PIDbuf[2];
}

/**
 * \par Function
 *    getPosPID
 * \par Description
 *    Get PID from Motor.
 * \param[in]
 *    p - P means Proportion.
 * \param[in]
 *    i - I means Integration.
 * \param[in]
 *    d - D means Differentiation.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void getPosPID(float * p,float * i,float * d, int idx)
{
//  uint8_t buf[8];
  uint8_t buf[12];
//  Wire.beginTransmission(MeEncoders[idx].address);
//  Wire.write(MeEncoders[idx]._slot);
//  Wire.write(CMD_GET_POS_PID);
//  Wire.endTransmission(0);
//  Wire.requestFrom(MeEncoders[idx].address,(uint8_t)12);

  uint8_t request_code = CMD_GET_POS_PID;
  uint8_t rlen = 12;
  request(&request_code, buf, rlen, idx);

//
//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }
//  *p = *(float*)buf;
//
//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }
//  *i = *(float*)buf;
//
//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }
//  *d = *(float*)buf;
  float *PIDbuf;
  PIDbuf = (float *)buf;
  *p = PIDbuf[0];
  *i = PIDbuf[1];
  *d = PIDbuf[2];
}

/**
 * \par Function
 *    getCurrentSpeed
 * \par Description
 *    The current speed of Motor's movement.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    The speed of encoder motor(The unit is rpm)
 * \par Others
 *    None
 */
float getCurrentSpeed(int idx)
{
  uint8_t buf[8];
  float speed;
//  Wire.beginTransmission(MeEncoders[idx].address);
//  Wire.write(MeEncoders[idx]._slot);
//  Wire.write(CMD_GET_SPEED);
//  Wire.endTransmission(0);
//  Wire.requestFrom(MeEncoders[idx].address,(uint8_t)4);
//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }
  uint8_t request_code = CMD_GET_SPEED;
  uint8_t rlen = 4;
  request(&request_code, buf, rlen, idx);
  speed = *(float*)buf;
  return speed;
}

/**
 * \par Function
 *    sendCmd(idx)
 * \par Description
 *    Send command to Motor.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void sendCmd(int idx)
{
//  Wire.beginTransmission(MeEncoders[idx].address);
//  for(int i=0;i<CmdBufLength;i++)
//    Wire.write(MeEncoders[idx].cmdBuf[i]);
//  Wire.endTransmission();
  I2C_write(MeEncoders[idx].cmdBuf, CmdBufLength, idx, Timeout);

}

/**
 * \par Function
 *    getRatio
 * \par Description
 *    Get the ratio of Motor.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    Return the ratio of Motor.
 * \par Others
 *    None
 */
float getRatio(int idx)
{
  uint8_t buf[8];
  float ratio;
//  Wire.beginTransmission(MeEncoders[idx].address);
//  Wire.write(MeEncoders[idx]._slot);
//  Wire.write(CMD_GET_RATIO);
//  Wire.endTransmission(0);
//  Wire.requestFrom(MeEncoders[idx].address,(uint8_t)4);
//  for(int i=0;i<4;i++)
//  {
//    buf[i] = Wire.read();
//  }

  uint8_t request_code = CMD_GET_RATIO;
  uint8_t rlen = 4;
  request(&request_code, buf, rlen, idx);
  ratio = *(float*)buf;
  return ratio;
}

/**
 * \par Function
 *    setRatio
 * \par Description
 *    Set the ratio to Motor.
 * \param[in]
 *    ratio - the ratio of Motor
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setRatio(float ratio, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_SET_RATIO;
  memcpy(MeEncoders[idx].cmdBuf+2,&ratio,4);
  sendCmd(idx);
}

/**
 * \par Function
 *    getPulse
 * \par Description
 *    Get the pulse of Motor.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    Return the pulse of Motor.
 * \par Others
 *    None
 */
int getPulse(int idx)
{
  uint8_t buf[8];
  int pulse;
//  Wire.beginTransmission(MeEncoders[idx].address);
//  Wire.write(MeEncoders[idx]._slot);
//  Wire.write(CMD_GET_PULSE);
//  Wire.endTransmission(0);
//  Wire.requestFrom(MeEncoders[idx].address,(uint8_t)2);
//  for(int i=0;i<2;i++)
//  {
//    buf[i] = Wire.read();
//  }

  uint8_t request_code = CMD_GET_PULSE;
  uint8_t rlen = 2;
  request(&request_code, buf, rlen, idx);

  pulse = *(int*)buf;
  return pulse;
}

/**
 * \par Function
 *    setPulse
 * \par Description
 *    Set the pulse to Motor.
 * \param[in]
 *    pulse - the line number of Motor
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setPulse(int pulse, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_SET_PULSE;
  memcpy(MeEncoders[idx].cmdBuf+2,&pulse,2);
  sendCmd(idx);
}

/**
 * \par Function
 *    setDevid
 * \par Description
 *    Set the devid to Motor.
 * \param[in]
 *    devid - the I2C adress
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void setDevid(uint8_t devid, int idx)
{
  MeEncoders[idx].cmdBuf[0] = MeEncoders[idx]._slot;
  MeEncoders[idx].cmdBuf[1] = CMD_SET_DEVID;
  MeEncoders[idx].cmdBuf[2] = devid;
  sendCmd(idx);
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
 * \param[in]
 *    lock_state - The lock state of Motor.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void runSpeedAndTime(float speed, float time, float lock_state, int idx)
{
  if(MeEncoders[idx]._lastTime == 0)
  {
    MeEncoders[idx]._lastTime = HAL_GetTick();
    runSpeed(speed,lock_state, idx);
  }

  if(HAL_GetTick() - MeEncoders[idx]._lastTime > (1000 * time))
  {
    MeEncoders[idx]._lastTime = 0;
    runSpeed(0,lock_state, idx);
  }
}

/**
 * \par Function
 *    isTarPosReached
 * \par Description
 *    Check whether the target position has been reached
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    true - The target position reaches
 *    false - Does not reach the target position
 * \par Others
 *    None
 */
bool isTarPosReached(int idx)
{
  uint8_t buf[8];
  bool lock_state;
//  Wire.beginTransmission(MeEncoders[idx].address);
//  Wire.write(MeEncoders[idx]._slot);
//  Wire.write(CMD_GET_LOCK_STATE);
//  Wire.endTransmission(0);
//
//  Wire.requestFrom(MeEncoders[idx].address,(uint8_t)2);
//  for(int i=0;i<2;i++)
//  {
//    buf[i] = Wire.read();
//  }

  uint8_t request_code = CMD_GET_LOCK_STATE;
  uint8_t rlen = 2;
  request(&request_code, buf, rlen, idx);

  lock_state = *(bool*)buf;
  return lock_state;
}

/**
 * \par Function
 *   getFirmwareVersion
 * \par Description
 *   Get Firmware Version, Only support EncodeDriver-V2.1.0 module firmware
 * \param[in]
 *   None
 * \par Output
 *   buffer: for storage version, length greater than 8
 * \return
 *   None
 * \par Others
 */
void getFirmwareVersion(uint8_t *buffer, int idx)
{
//  Wire.beginTransmission(MeEncoders[idx].address);
//  Wire.write(MeEncoders[idx]._slot);
//  Wire.write(CMD_GET_FIRWARE_VERSION);
//  Wire.endTransmission(0);
//  Wire.requestFrom(MeEncoders[idx].address,(uint8_t)8);
//  for(int i=0;i<8;i++)
//  {
//    buffer[i] = Wire.read();
//  }
  uint8_t request_code = CMD_GET_FIRWARE_VERSION;
  uint8_t rlen = 8;
  request(&request_code, buffer, rlen, idx);
}


void request(uint8_t *request_code, uint8_t *readData, int rlen, int idx) {
  I2C_write(&MeEncoders[idx]._slot, 1, idx, Timeout);
  I2C_write(request_code, 1, idx, Timeout);
  I2C_read(readData, rlen, idx, Timeout);
  
}

void I2C_write(uint8_t *writeData, int wlen, int idx, uint32_t timeout) {
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Master_Transmit(&hi2c1, (MeEncoders[idx].address) << 1, writeData, wlen, timeout);
  if (status != HAL_OK) {
    printf("I2C master transmit error.\n");
  }

}

void I2C_read( uint8_t *readData, int rlen, int idx, uint32_t timeout) {
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Master_Receive(&hi2c1, ((MeEncoders[idx].address) << 1) | 1, readData, rlen, timeout);
  if (status != HAL_OK) {
    printf("I2C master receive error.\n");
  }
}

void MeEncoderNew_Init() {
	MeEncoderNewSlot(SLOT1, 0);
	MeEncoderNewSlot(SLOT2, 1);
	
	reset(0);
	reset(1);

}
