/**
 ******************************************************************************
 * @file    rgbdisplay.c
 * @author
 * @version V1.0.0
 * @date    21-May-2015
 * @brief   RGB LED Application!
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2019 OPPLE Inc.
 *
 ******************************************************************************
 */

#include "mico.h"
#include "rgbdisplay.h"

uint8_t circle = 0;
rgb_display_context_t   *app_context = NULL;

extern struct cli_command built_user[];

//void gclkTrigger_thread(uint32_t inContext);

int application_start( void )
{
  OSStatus  err = kNoErr;
  mico_Context_t    *mico_context;

  /* Create application context */
  app_context = (rgb_display_context_t *) calloc( 1, sizeof(rgb_display_context_t) );
  require_action( app_context, exit, err = kNoMemoryErr );

  /* Create mico system context and read application's config data from flash */
  mico_context = mico_system_context_init( sizeof(rgb_display_config_t) );
  app_context->flashConfig = mico_system_context_get_user_data( mico_context );

  /* Start MiCO system functions according to mico_config.h*/
  mico_system_init( mico_system_context_init( 0 ) );
  
  /* Output on debug serial port */
  os_rgb_log( "rgbdisplay start!" );

  opple_cli_register();
  MicoPwmInitialize(MICO_PWM_3, 500000, 50);
  //MicoPwmStop(MICO_PWM_3);
  //mico_wlan_start_monitor();
  //mico_wlan_monitor_set_channel(1);

  GPIO_RGB_init();
  //mico_nanosecond_init();
  myNsDelay(100);

  /*GCLK trigger thread */
  /*
  err = mico_rtos_create_thread( NULL, MICO_APPLICATION_PRIORITY, "gclk trigger", gclkTrigger_thread,
                                 STACK_SIZE_OTA_UPDATE_THREAD, (mico_thread_arg_t)app_context );
  require_noerr_action( err, exit, os_rgb_log("ERROR: Unable to start the gclk trigger thread.") );
*/
  uint16_t i,bits[4]={0xff,0x5ff,0xaff,0xfff};
/*
  MicoPwmStart(MICO_PWM_3);
  while(1)
  {
      mico_thread_sleep(2);
  }
*/

  MBI_init();
  while(1)
  {
      //MicoGpioOutputTrigger( MICO_SYS_LED );

      //register_config();
      //mico_thread_msleep(1);
      RGB_SDI_R_schedule(bits, 32);
      circle++;
      os_rgb_log( "while" );
      mico_thread_msleep(200);
  }

exit:
  //if ( wait_wifi_sem != NULL )
      //mico_rtos_deinit_semaphore( &wait_wifi_sem );
  mico_rtos_delete_thread(NULL);
  return err;
}


