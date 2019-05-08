/*
 * rgbdisplay.h
 *
 *  Created on: 2019��4��1��
 *      Author: OP056666
 */

#ifndef RGBDISPLAY_RGBDISPLAY_H_
#define RGBDISPLAY_RGBDISPLAY_H_

#define CHIP_SIZE                           16
#define STACK_SIZE_OTA_UPDATE_THREAD        0x1000

typedef enum
{
    G = 1<<0,
    R = 1<<1,
    B = 1<<2,
}RGB_Type;


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