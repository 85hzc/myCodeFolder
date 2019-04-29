/**
 ******************************************************************************
 * @file    hello_world.c
 * @author  William Xu
 * @version V1.0.0
 * @date    21-May-2015
 * @brief   First MiCO application to say hello world!
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 */

#include "gpioi2c.h"
#include "mico.h"
#include "siliconSi115x.h"

bool INT = 0;
extern struct cli_command built_user[];

SILAB_SENSOR_StatusTypeDef siliconSensor_IO_Read(uint8_t* rxBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead);
Si115xSample_t samples; //Stores the sample data from reading the sensor


/* I2C device */
mico_i2c_device_t silab_i2c_device = {
    SILAB_I2C_PORT, SI1153_I2C_ADDR, I2C_ADDRESS_WIDTH_7BIT, I2C_STANDARD_SPEED_MODE
};

void i2cmsginfo(mico_i2c_message_t hts221_i2c_msg)
{
    int i;
    uint8_t *pt,*pr;

    pt= (uint8_t *)hts221_i2c_msg.tx_buffer;
    pr= (uint8_t *)hts221_i2c_msg.rx_buffer;
    printf("---------------i2cmsginfo-------------\r\n");
    if (pt != NULL) {
        printf("TX:");
        for(i=0;i<hts221_i2c_msg.tx_length;i++) {
            printf(" 0x%x", pt[i]);
        }
        printf("\r\n");
    }
    if (pr != NULL) {
        printf("RX:");
        for(i=0;i<hts221_i2c_msg.rx_length;i++) {
            printf(" 0x%x", pr[i]);
        }
        printf("\r\n");
    }
    printf("--------------------------------------\r\n");
}

void _si1153_low_irq_handler( void* arg )
{
    SILAB_log("_si1153_low_irq_handler");
    INT = 1;
}
#if 1
int16_t _sendCmd(uint8_t command);
#ifndef GPIO_I2C
SILAB_SENSOR_StatusTypeDef siliconSensor_IO_Init(void)
{
  // I2C init
  MicoI2cFinalize(&silab_i2c_device);   // in case error
  MicoI2cInitialize(&silab_i2c_device);
  if( false == MicoI2cProbeDevice(&silab_i2c_device, 5) ){
    SILAB_log("SILAB_ERROR: no i2c device found!");
    return SILAB_SENSOR_ERROR;
  }
  return SILAB_SENSOR_OK;
}

SILAB_SENSOR_StatusTypeDef siliconSensor_IO_Write(uint8_t *pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
  mico_i2c_message_t hts221_i2c_msg = {NULL, NULL, 0, 0, 0, false};
  int iError = 0;
  uint8_t array[8];
  uint8_t stringpos;

  array[0] = RegisterAddr | 0x40;
  for (stringpos = 0; stringpos < NumByteToWrite; stringpos++) {
    array[stringpos + 1] = *(pBuffer + stringpos);
  }

  SILAB_log("siliconSensor_IO_Write address=0x%x",RegisterAddr);
  //iError = MicoI2cBuildTxMessage(&hts221_i2c_msg, array, NumByteToWrite + 2, 3);
  iError = MicoI2cBuildTxMessage(&hts221_i2c_msg, array, NumByteToWrite + 1, 3);
  if(0 != iError){
    SILAB_log("SILAB_SENSOR_ERROR");
    iError = SILAB_SENSOR_ERROR;
  }
  iError = MicoI2cTransfer(&silab_i2c_device, &hts221_i2c_msg, 1);
  if(0 != iError){
    SILAB_log("SILAB_SENSOR_ERROR");
    iError = SILAB_SENSOR_ERROR;
  }
  i2cmsginfo(hts221_i2c_msg);
  return (SILAB_SENSOR_StatusTypeDef)iError;
}

SILAB_SENSOR_StatusTypeDef siliconSensor_IO_Write_block(uint8_t *pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
  mico_i2c_message_t hts221_i2c_msg = {NULL, NULL, 0, 0, 0, false};
  int iError = 0;
  uint8_t array[8];
  uint8_t stringpos;

  SILAB_log("siliconSensor_IO_Write_block address=0x%x",RegisterAddr);

  array[0] = RegisterAddr;
  for (stringpos = 0; stringpos < NumByteToWrite; stringpos++) {
    array[stringpos + 1] = *(pBuffer + stringpos);
  }

  iError = MicoI2cBuildTxMessage(&hts221_i2c_msg, array, NumByteToWrite + 1, 3);
  if(0 != iError){
    SILAB_log("SILAB_SENSOR_ERROR");
    iError = SILAB_SENSOR_ERROR;
  }
  iError = MicoI2cTransfer(&silab_i2c_device, &hts221_i2c_msg, 1);
  if(0 != iError){
    SILAB_log("SILAB_SENSOR_ERROR");
    iError = SILAB_SENSOR_ERROR;
  }
  i2cmsginfo(hts221_i2c_msg);
  return (SILAB_SENSOR_StatusTypeDef)iError;
}

SILAB_SENSOR_StatusTypeDef siliconSensor_IO_Read(uint8_t* rxBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  mico_i2c_message_t hts221_i2c_msg = {NULL, NULL, 0, 0, 0, false};
  int iError = 0;
  uint8_t array[8] = {0};
  array[0] = RegisterAddr | 0x40;

  SILAB_log("siliconSensor_IO_Read address=0x%x",RegisterAddr);

  iError = MicoI2cBuildCombinedMessage(&hts221_i2c_msg, array, rxBuffer, 1, NumByteToRead, 3);
  if(0 != iError){
    SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
  iError = MicoI2cTransfer(&silab_i2c_device, &hts221_i2c_msg, 1);
  if(0 != iError){
    SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
  i2cmsginfo(hts221_i2c_msg);
  return (SILAB_SENSOR_StatusTypeDef)iError;
}

SILAB_SENSOR_StatusTypeDef siliconSensor_IO_Read_block(uint8_t* rxBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  mico_i2c_message_t hts221_i2c_msg = {NULL, NULL, 0, 0, 0, false};
  int iError = 0;
  uint8_t array[8] = {0};
  array[0] = RegisterAddr;

  SILAB_log("siliconSensor_IO_Read_block address=0x%x",RegisterAddr);

  iError = MicoI2cBuildCombinedMessage(&hts221_i2c_msg, array, rxBuffer, 1, NumByteToRead, 3);
  if(0 != iError){
    SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
  iError = MicoI2cTransfer(&silab_i2c_device, &hts221_i2c_msg, 1);
  if(0 != iError){
    SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
  i2cmsginfo(hts221_i2c_msg);
  return (SILAB_SENSOR_StatusTypeDef)iError;
}
#endif

int16_t Si115xBlockWrite(uint8_t address, uint8_t length, uint8_t * data)
{

    SILAB_log("Si115xBlockWrite address=0x%x",address);

    //siliconSensor_IO_Write_block(data, SI1153_I2C_ADDR, address, length);
    //siliconSensor_IO_Read(&data, SI1153_I2C_ADDR, RegisterAddr, 1);
    SILAB_IO_Write_Block(data, SI1153_I2C_ADDR, address, length);

    return 0;
}
/*
int16_t Si115xWriteToRegister(uint8_t RegisterAddr, uint8_t command)
{
    SILAB_log("RegisterAddr=0x%x",RegisterAddr);

    siliconSensor_IO_Write(&command, SI1153_I2C_ADDR, RegisterAddr, 1);
    //siliconSensor_IO_Read(&data, SI1153_I2C_ADDR, RegisterAddr, 1);

    return 0;
}
*/
int16_t Si115xParamRead(uint8_t address)
{
  // returns Parameter[address]
  int16_t retval;
  uint8_t cmd = 0x40 + (address & 0x3F);

  SILAB_log("Si115xParamRead address=0x%x",address);

  retval=_sendCmd(cmd);
  if( retval != 0 )
  {
    return retval;
  }
  retval = Si115xReadFromRegister(SI115x_REG_RESPONSE1);
  return retval;
}


int siliconSensorInit( void )
{
    /* Configure the low level interface */
    if(siliconSensor_IO_Init() != SILAB_SENSOR_OK)
    {
        return SILAB_SENSOR_ERROR;
    }

    return SILAB_SENSOR_OK;
}

static int16_t _waitUntilSleep()
{
  int16_t retval = -1;
  uint8_t count = 0;
  // This loops until the Si115x is known to be in its sleep state
  // or if an i2c error occurs
  while(count < 5)
  {
    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    if((retval&SI115x_RSP0_CHIPSTAT_MASK) == SI115x_RSP0_SLEEP)
      break;
    if(retval <  0)
      return retval;
    count++;
  }
  return 0;
}

int16_t _sendCmd(uint8_t command)
{
  int16_t  response;
  int8_t   retval;
  uint8_t  count = 0;

  // Get the response register contents
  response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  if(response < 0)
  {
    return response;
  }

  response = response & SI115x_RSP0_COUNTER_MASK;

  // Double-check the response register is consistent
  while(count < 5)
  {
    if((retval = _waitUntilSleep()) != 0)
      return retval;

    if(command == 0)
      break; // Skip if the command is NOP

    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);

    if((retval&SI115x_RSP0_COUNTER_MASK) == response)
      break;
    else if(retval < 0)
      return retval;
    else
      response = retval & SI115x_RSP0_COUNTER_MASK;

    count++;
  } // end loop

  // Send the Command
  if(retval = (Si115xWriteToRegister(SI115x_REG_COMMAND, command, 1))!= 0)
  {
    return retval;
  }

  count = 0;
  // Expect a change in the response register
  while(count < 5)
  {
    if(command == 0)
      break; // Skip if the command is NOP

    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    if((retval & SI115x_RSP0_COUNTER_MASK) != response)
      break;
    else if(retval < 0)
      return retval;

    count++;
  } // end loop

  return 0;
}


/**************************************************************************//**
 * @brief Read block of data from Si117x i2c.
 *****************************************************************************/
int16_t Si115xBlockRead(uint8_t address, uint8_t length, uint8_t *data)
{
  SILAB_log("Si115xBlockRead address=0x%x",address);

  SILAB_IO_Read_Block(data, SI1153_I2C_ADDR, address, length);

  return((int) 0);
}

int16_t Si115xParamSet(uint8_t address, uint8_t value)
{
  int16_t retval;
  uint8_t buffer[2];
  int16_t response_stored;
  int16_t response;

  SILAB_log("Si115xParamSet address=0x%x",address);

  retval = _waitUntilSleep();
  if(retval !=0)
  {
    return retval;
  }

  response_stored = SI115x_RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(SI115x_REG_RESPONSE0);

  buffer[0] = value;
  buffer[1] = 0x80 + (address & 0x3F);

  retval = Si115xBlockWrite(SI115x_REG_HOSTIN0,
                            2,
                            (uint8_t*) buffer);
  if(retval != 0)
    return retval;

  // Wait for command to finish
  response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  while((response & SI115x_RSP0_COUNTER_MASK) == response_stored)
  {
    response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  }

  if(retval < 0)
    return retval;
  else
    return 0;
}

int16_t Si115xReset()
{
  int16_t retval = 0;

  SILAB_log("------------Si115xReset-------------\r\n");

  // Do not access the Si115x earlier than 25 ms from power-up.
  // Uncomment the following lines if Si115xReset() is the first
  // instruction encountered, and if your system MCU boots up too
  // quickly.
  mico_thread_msleep(10);
  mico_thread_msleep(10);
  mico_thread_msleep(10);

  // Perform the Reset Command
  retval += Si115xWriteToRegister(SI115x_REG_COMMAND, 1, 1);

  // Delay for 10 ms. This delay is needed to allow the Si115x
  // to perform internal reset sequence.
  mico_thread_msleep(10);

  return retval;
}
#if 1
// ch0: prox, large IR photodiode, 97us integration time, low signal range, LED2 = 321mA, LED1 = LED3 = none, accumulate 1, no right shift
int16_t Si115xInitLongRangeProx(  )
{
    int16_t    retval;

    SILAB_log("------------Si115xInitLongRangeProx 3cho-------------\r\n");

    retval  = Si115xReset( );
    mico_thread_msleep(100);

    retval += Si115xParamSet( SI115x_PARAM_CH_LIST, 0x07);

    retval += Si115xParamSet( SI115x_PARAM_LED1_A, 0x3f);
    retval += Si115xParamSet( SI115x_PARAM_LED2_A, 0x3f);
    retval += Si115xParamSet( SI115x_PARAM_LED3_A, 0x3f);

    retval += Si115xParamSet( SI115x_PARAM_ADCCONFIG0, 0x62);
    retval += Si115xParamSet( SI115x_PARAM_MEASCONFIG0, 0x01);
    retval += Si115xParamSet( SI115x_PARAM_ADCCONFIG1, 0x62);
    retval += Si115xParamSet( SI115x_PARAM_MEASCONFIG1, 0x02);
    retval += Si115xParamSet( SI115x_PARAM_ADCCONFIG2, 0x62);
    retval += Si115xParamSet( SI115x_PARAM_MEASCONFIG2, 0x04);

    retval += Si115xWriteToRegister( SI115x_REG_IRQ_ENABLE, 0x07, 1);
    //retval += Si115xWriteToRegister( SI115x_REG_IRQ_ENABLE, 0);

    return retval;
}

int16_t Si115xInitProxAls( bool proxOnly )
{
    int16_t    retval;

    retval  = Si115xReset( );
    mico_thread_msleep(100);

    if (proxOnly) // prox only, no als
    {
        retval += Si115xParamSet(  SI115x_PARAM_LED1_A, 0x3f);
        retval += Si115xParamSet(  SI115x_PARAM_CH_LIST, 0x01);
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG0, 0x62);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS0, 0x80);//80
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST0, 0x40);
        retval += Si115xParamSet(  SI115x_PARAM_MEASCONFIG0, 0x21);
        retval += Si115xWriteToRegister(  SI115x_REG_IRQ_ENABLE, 0x01);
    }
    else // prox + als
    {
        retval += Si115xParamSet(  SI115x_PARAM_LED1_A, 0x3f); // LED1
        retval += Si115xParamSet(  SI115x_PARAM_CH_LIST, 0x0f);
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG0, 0x62);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS0, 0x80);
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST0, 0x40);
        retval += Si115xParamSet(  SI115x_PARAM_MEASCONFIG0, 0x21); //LED1
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG1, 0x4d);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS1, 0xe1);
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST1, 0x40);
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG2, 0x41);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS2, 0xe1);
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST2, 0x50);
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG3, 0x4d);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS3, 0x87);
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST3, 0x40);
        retval += Si115xWriteToRegister(  SI115x_REG_IRQ_ENABLE, 0x0f);
    }

    return retval;
}

#else
// ch0: prox, large IR photodiode, 97us integration time, low signal range, LED2 = 321mA, LED1 = LED3 = none, accumulate 1, no right shift
int16_t Si115xInitLongRangeProx( )
{
    int16_t    retval;

    SILAB_log("------------Si115xInitLongRangeProx 1cho-------------\r\n");

    retval  = Si115xReset( );
    mico_thread_msleep(100);
    retval += Si115xParamSet(  SI115x_PARAM_LED2_A, 0x3f);
    retval += Si115xParamSet(  SI115x_PARAM_CH_LIST, 0x01);
    retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG0, 0x62);
    retval += Si115xParamSet(  SI115x_PARAM_ADCSENS0, 0x02);
    retval += Si115xParamSet(  SI115x_PARAM_ADCPOST0, 0x40);
    retval += Si115xParamSet(  SI115x_PARAM_MEASCONFIG0, 0x32);
    retval += Si115xWriteToRegister(  SI115x_REG_IRQ_ENABLE, 0x01);

    return retval;
}
#endif

/*
U8 measurementPaused = 0;
void pauseMeasurement(void) {

   if (measurementPaused)
       return;

   WriteToRegister(REG_IRQ_ENABLE, 0);

   //to disable any Si115x interrupts
   // Need to make sure the machine is paused
   // the error condition in response register is asynchronous
   while (1) {
       // Keep sending RESET_CMD_CTR command until the response is zero
       while (1)
       {
           if ( GetResponse() == 0)
               break;
           else
               RESET_CMD_CTR();
       }
   }

   // Pause the device
   Si115xPause();

   // Wait for response
   while(1)
   {
       if (GetResponse() != 0)
           break;
   }

   // When the Si115xPause () response is good, we expect it to be a '1'.
   if (GetResponse() == 1)
       break; // otherwise, start over.
   }
   measurementPaused = 1;
}

void resumeMeasurement(void) {

    if (!measurementPaused)
        return;

    ClearIrqStatus();
    Si115xWriteToRegister( si115x_handle, REG_IRQ_ENABLE, 0x07);    // re-enables INT
    Si115xStart ();
    measurementPaused = 0;
}
*/
void Si115xForce(void)
{
    SILAB_log("------------Si115xForce-------------\r\n");
    _sendCmd(0x11);
}

void Si115xStart (void)
{
    SILAB_log("------------Si115xStart-------------\r\n");
    _sendCmd(0x13);
}

void Si115xHandler()
{
    uint8_t buffer[13];

    memset(buffer, 0, sizeof(buffer));

    SILAB_log("------------Si115xHandler-------------\r\n");

    Si115xBlockRead( SI115x_REG_IRQ_STATUS,
                      13,
                      buffer);
    samples.irq_status = buffer[0];
    samples.ch0  = buffer[1] << 16;
    samples.ch0 |= buffer[2] <<  8;
    samples.ch0 |= buffer[3];
    if( samples.ch0 & 0x800000 )       //Sign extending samples.ch0
        samples.ch0 |= 0xFF000000;

    samples.ch1  = buffer[4] << 16;
    samples.ch1 |= buffer[5] <<  8;
    samples.ch1 |= buffer[6];
    if( samples.ch1 & 0x800000 )
        samples.ch1 |= 0xFF000000;

    samples.ch2  = buffer[7] << 16;
    samples.ch2 |= buffer[8] <<  8;
    samples.ch2 |= buffer[9];
    if( samples.ch2 & 0x800000 )
        samples.ch2 |= 0xFF000000;

    samples.ch3  = buffer[10] << 16;
    samples.ch3 |= buffer[11] <<  8;
    samples.ch3 |= buffer[12];
    if( samples.ch3 & 0x800000 )
        samples.ch3 |= 0xFF000000;

    SILAB_log("-------------ch1:%d,ch2:%d,ch3:%d,ch4:%d-------------\r\n",samples.ch0,samples.ch1,samples.ch2,samples.ch3);
}

void getSensorData(void)
{
    if ( INT == 1)
    {
        INT = 0;
        SILAB_log("-----------------------INT START---------------------\r\n");
        // Sensor data ready
        // Process measurement
        Si115xHandler();

        // Start next measurement
        Si115xForce();
        SILAB_log("------------------------INT END----------------------\r\n");
    }
}

void getSensorDataByHostout(void)
{
    uint32_t CH1_PS,CH2_PS,CH3_PS;

    SILAB_log("------------getSensorDataByHostout-------------\r\n");
    CH1_PS = Si115xReadFromRegister (SI115x_REG_HOSTOUT1) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT0);
    CH2_PS = Si115xReadFromRegister (SI115x_REG_HOSTOUT3) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT2);
    CH3_PS = Si115xReadFromRegister (SI115x_REG_HOSTOUT5) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT4);

    SILAB_log("-------------ch1:%d,ch2:%d,ch3:%d-------------\r\n",CH1_PS,CH2_PS,CH3_PS);
    //Si115xForce();
}

void DEMO_Init()
{
    uint8_t data;

    //Get HW ID of the part and use that to decide what we are talking to
    data = Si115xReadFromRegister( SI1153_REG_HW_ID);

    switch(data){
        case SI1153_PROX_HW_ID:     //In case 0, this could be multiple sensors, but we assume PROX and declare if the switch is set to another sensor then there is unpredictable behavior.
            Si115xInitProxAls(false);
            break;
        case SI1153_LR_PROX_HW_ID:
            Si115xInitLongRangeProx();
            break;
    }
    //Si115xInitLongRangeProx();// init for this project.hzc
    Si115xForce();
}

void DEMO_LongRangeProx (void)
{
    uint32_t result;
    uint16_t mag;

    getSensorData();

    result = (uint32_t) samples.ch0;

    SILAB_log( "ch0 result=%x",result );
/*
    if (result >= 0x7FFFFF)
    {
        // Overflow occurred
        sprintf(ch0Str, "  PROX     OVERFLOW");
        drawScreenText(ch0Str, 116, FONT_SCALE_DEFAULT);
        mag = 128;
    }
    else
    {
        mag = (uint16_t) result;

        // Update screen
        sprintf(ch0Str, "  PROX %5u COUNTS", mag);
        drawScreenText(ch0Str, 116, FONT_SCALE_DEFAULT);

        mag = scaleSensorData(mag, 10);
    }

    drawScreenSquare(mag);
*/
}

#endif

int application_start( void )
{

  /* Start MiCO system functions according to mico_config.h*/
  mico_system_init( mico_system_context_init( 0 ) );
  
  /* Output on debug serial port */
  SILAB_log( "Silicon lib sensor system!" );

  opple_cli_register();

/*
  extern const platform_i2c_t platform_i2c_peripherals[];
  int ddd;
  for(ddd=0;ddd<9;ddd++) {
      //MicoGpioOutputTrigger(platform_i2c_peripherals[MICO_I2C_1].pin_scl->pin);
      SILAB_log( "pinpinpin" );
      MicoGpioOutputHigh(platform_i2c_peripherals[MICO_I2C_1].pin_scl->pin);
      mico_thread_msleep(10);
      MicoGpioOutputLow(platform_i2c_peripherals[MICO_I2C_1].pin_scl->pin);
      mico_thread_msleep(10);
  }
*/
#ifndef GPIO_I2C
  SILAB_IO_Init();
  mico_thread_msleep(200);
#endif

#if 0
#if 1 //init IRQ pin
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_14, INPUT_PULL_UP );
  MicoGpioEnableIRQ( (mico_gpio_t)MICO_GPIO_14, IRQ_TRIGGER_FALLING_EDGE, _si1153_low_irq_handler, NULL );
#else
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_12, INPUT_PULL_DOWN );
  MicoGpioEnableIRQ( (mico_gpio_t)MICO_GPIO_13, IRQ_TRIGGER_RISING_EDGE, _si1153_low_irq_handler, NULL );
#endif
#endif
  DEMO_Init();
  mico_thread_msleep(200);

/*
  SILAB_log("----------------------------------------------\r\n");
  Si115xWriteToRegister(SI115x_REG_COMMAND, SI115x_CMD_RESET);
  mico_thread_msleep(100);
  SILAB_log("----------------SI115x_CMD_RESET--------------\r\n");

  SILAB_ReadID(NULL);
  mico_thread_msleep(100);
  SILAB_log("----------------SI115x_REG_HW_ID--------------\r\n");

  Si115xReadFromRegister(SI115x_REG_PART_ID);
  mico_thread_msleep(100);
  SILAB_log("----------------SI115x_REG_PART_ID--------------\r\n");

  Si115xReadFromRegister(SI115x_REG_REV_ID);
  mico_thread_msleep(100);
  SILAB_log("----------------SI115x_REG_REV_ID--------------\r\n");
*/
  SILAB_log("------------while-------------\r\n");
  while(1)
  {
      //SILAB_ReadID(NULL);
      //mico_thread_msleep(500);

      //getSensorData();

      //getSensorDataByHostout();
      //SILAB_log("------------mico_thread_sleep(2)-------------\r\n");
      mico_thread_sleep(2);
  }

}


