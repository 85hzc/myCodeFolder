/*
 * rgbdisplay.h
 *
 *  Created on: 2019Äê4ÔÂ1ÈÕ
 *      Author: OP056666
 */

#ifndef RGBDISPLAY_RGBDISPLAY_H_
#define RGBDISPLAY_RGBDISPLAY_H_

#define MBI5153_SIZE                        1
#define STACK_SIZE_OTA_UPDATE_THREAD        0x1000

#define os_rgb_log(format, ...)  custom_log("rgbdisplay", format, ##__VA_ARGS__)

/*Application's configuration stores in flash*/
typedef struct
{
  bool              isClouldAuth;
  char              SN[8];

} rgb_display_config_t;


typedef struct _app_context_t
{
  /*Running status*/
  bool              informFlag;
  uint8_t           mac[6];

  /*Flash content*/
  rgb_display_config_t  *flashConfig;
} rgb_display_context_t;

#endif /* RGBDISPLAY_RGBDISPLAY_H_ */
