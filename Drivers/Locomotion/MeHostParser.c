/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class   MeHostParser
 * \brief   Driver for Me Host Parser module.
 * @file    MeHostParser.cpp
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/11/12
 * @brief   Driver for Me Host Parser module.
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
 * This file is a drive for Me Host Parser device, The Me Host Parser inherited the
 * MeSerial class from SoftwareSerial.
 *
 * \par Method List:
 *
 *    1. uint8_t pushStr(uint8_t * str, uint32_t length);
 *    2. uint8_t pushByte(uint8_t ch);
 *    3. uint8_t run();
 *    4. uint8_t getPackageReady();
 *    5. uint8_t getData(uint8_t *buf, uint32_t size);
 *    6. void print(char *str, uint32_t * cnt);
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * forfish         2015/11/12    1.0.0            Add description
 * </pre>
 *
 */

#include <stddef.h>
#include <memory.h>
#include <stdlib.h>
#include "MeHostParser.h"

#define HEAD    0xA5
#define TAIL    0x5A

//  states
#define ST_WAIT_4_START     0x01
#define ST_HEAD_READ        0x02
#define ST_MODULE_READ      0x03
#define ST_LENGTH_READ      0x04
#define ST_DATA_READ        0x05
#define ST_CHECK_READ       0x06

/**
 * Alternate Constructor which can call your own function to map the Host Parser to arduino port,
 * no pins are used or initialized here.
 * \param[in]
 *   None
*/
void MeHostParserInit(MeParser *parser)
{
  parser->state = ST_WAIT_4_START;
  parser->in = 0;
  parser->out = 0;
  parser->packageReady = 0;

  parser->module = 0;
  parser->length = 0;
  parser->data = NULL;
  parser->check = 0;

  parser->lengthRead = 0;
  parser->currentDataPos = 0;
}

/**
 * \par Function
 *    getPackageReady
 * \par Description
 *    Get the package ready state.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    Return the status ready or not.
 * \par Others
 *    None
 */
uint8_t getPackageReady(MeParser *parser)
{
  return (1 == parser->packageReady);
}

/**
 * \par Function
 *    pushStr
 * \par Description
 *    To push a string to Host Parser.
 * \param[in]
 *    str - A pointer to a string.
 * \param[in]
 *    length - The length of the string.
 * \par Output
 *    None
 * \par Return
 *    Return the index of pushing string.
 * \par Others
 *    None
 */
uint8_t pushStr(MeParser *parser, uint8_t * str, uint32_t length)
{
  if (length > ((parser->in + BUF_SIZE - parser->out - 1) & MASK))
  {
    return 0;
  }
  else
  {
    for (int i = 0; i < length; ++i)
    {
      pushByte(parser, str[i]);
    }
  }
}

/**
 * \par Function
 *    pushByte
 * \par Description
 *    To push a byte to Host Parser.
 * \param[in]
 *    ch - A pointer to a char.
 * \par Output
 *    None
 * \par Return
 *    Return the status of pushing char.
 * \par Others
 *    None
 */
uint8_t pushByte(MeParser *parser, uint8_t ch)
{
  if (((parser->in + 1) & MASK) != parser->out)
  {
    parser->buffer[parser->in] = ch;
    ++parser->in;
    parser->in &= MASK;
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * \par Function
 *    getByte
 * \par Description
 *    To get a byte from Host Parser.
 * \param[in]
 *    ch - A pointer to a char.
 * \par Output
 *    None
 * \par Return
 *    Return the status of getting char.
 * \par Others
 *    None
 */
uint8_t getByte(MeParser *parser, uint8_t * ch)
{
  if (parser->in != parser->out)
  {
    *ch = parser->buffer[parser->out];
    ++parser->out;
    parser->out &= MASK;
    return 1;
  }
  else
  {
    // Serial.println("GET error!");
    return 0;
  }
}

/**
 * \par Function
 *    calculateLRC
 * \par Description
 *    To calculate the LRC.
 * \param[in]
 *    data - A pointer to data.
 * \param[in]
 *    length - The length of LRC.
 * \par Output
 *    None
 * \par Return
 *    Return the LRC.
 * \par Others
 *    None
 */
    uint8_t calculateLRC(uint8_t *data, uint32_t length)
{
  uint8_t LRC = 0;
  for (uint32_t i = 0; i < length; ++i)
  {
    LRC ^= data[i];
  }
  return LRC;
}

/**
 * \par Function
 *    run
 * \par Description
 *    The running status of Host Parser.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    Return the status of Host Parser's running.
 * \par Others
 *    None
 */
uint8_t run(MeParser *parser)
{
  uint8_t ch = 0;
  while (getByte(parser, &ch))
  {
    switch (parser->state)
    {
      case ST_WAIT_4_START:
        if (HEAD == ch)
        {
          parser->state = ST_HEAD_READ;
        }
        break;
      case ST_HEAD_READ:
        parser->module = ch;
        parser->state = ST_MODULE_READ;
        break;
      case ST_MODULE_READ:
        //  read 4 bytes as "length"
        *(((uint8_t *)&parser->length) + parser->lengthRead) = ch;
        ++parser->lengthRead;
        if (4 == parser->lengthRead)
        {
          parser->lengthRead = 0;
          parser->state = ST_LENGTH_READ;
        }
        break;
      case ST_LENGTH_READ:
        //  alloc space for data
        if (0 == parser->currentDataPos)
        {
          if (parser->length > 255)
          {
            parser->state = ST_WAIT_4_START;
            parser->currentDataPos = 0;
            parser->lengthRead = 0;
            parser->length = 0;
            parser->module = 0;
            parser->check = 0;
            break;
          }
          parser->data = (uint8_t *)malloc(parser->length + 1);
          if (NULL == parser->data)
          {
            parser->state = ST_WAIT_4_START;
            parser->currentDataPos = 0;
            parser->lengthRead = 0;
            parser->length = 0;
            parser->module = 0;
            parser->check = 0;
            break;
          }
        }
        //  read data
        parser->data[parser->currentDataPos] = ch;
        ++parser->currentDataPos;
        if (parser->currentDataPos == parser->length)
        {
          parser->currentDataPos = 0;
          parser->state = ST_DATA_READ;
        }
        break;
      case ST_DATA_READ:
        parser->check = ch;
        if (parser->check != calculateLRC(parser->data, parser->length))
        {
          parser->state = ST_WAIT_4_START;
          if (NULL != parser->data)
          {
            free(parser->data);
            parser->data = NULL;
          }
          parser->currentDataPos = 0;
          parser->lengthRead = 0;
          parser->length = 0;
          parser->module = 0;
          parser->check = 0;
        }
        else
        {
          parser->state = ST_CHECK_READ;
        }
        break;
      case ST_CHECK_READ:
        if (TAIL != ch)
        {
          if (NULL != parser->data)
          {
            free(parser->data);
            parser->data = NULL;
          }
          parser->length = 0;
        }
        else
        {
          parser->packageReady = 1;
        }
        parser->state = ST_WAIT_4_START;
        parser->currentDataPos = 0;
        parser->lengthRead = 0;
        parser->module = 0;
        parser->check = 0;
        break;
      default:
        break;
    }
  }
  return parser->state;
}

/**
 * \par Function
 *    getData
 * \par Description
 *    Copy data to user's buffer.
 * \param[in]
 *    buf - A buffer for a getting data.
 * \param[in]
 *    size - The length of getting data.
 * \par Output
 *    None
 * \par Return
 *    Return the length of getting data or 0.
 * \par Others
 *    None
 */
uint8_t getData(MeParser *parser, uint8_t *buf, uint32_t size)
{
  int copySize = (size > parser->length) ? parser->length : size;
  if ((NULL != parser->data) && (NULL != buf))
  {
    memcpy(buf, parser->data, copySize);
    free(parser->data);
    parser->data = NULL;
    parser->length = 0;
    parser->packageReady = 0;

    return copySize;
  }
  else
  {
    return 0;
  }
}