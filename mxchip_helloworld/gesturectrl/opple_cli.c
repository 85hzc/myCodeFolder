/*
 * opple_cli.c
 *
 *  Created on: 2019Äê2ÔÂ15ÈÕ
 *      Author: OP056666
 */

#include "mico.h"

static void probe_request_command(char *pcWriteBuffer, int xWriteBufferLen,int argc, char **argv)
{
    int length;
    char buffer[256], mac[6]={'FC','EE','E6','12','71','AE'};
    static int16_t seq = 0;

    memset(buffer, 0, sizeof(buffer));
    length = 64;

    buffer[0] = 0x3;
    buffer[1] = 0;
    buffer[2] = 0;
    buffer[3] = 0;
    memset(&buffer[4], 0xff, 6);
    memset(&buffer[10], mac, 6);
    memset(&buffer[16], 0xff, 6);

    buffer[22] = seq++;
    buffer[23] = 0;

    buffer[24] = 0;//SSID
    buffer[25] = 6;//length
    memcpy(&buffer[26],"aabbcc",6);
    int i=0;
    do{
        mico_wlan_send_mgnt(buffer, length);
        mico_rtos_thread_msleep(100);
    }while(++i < 50);

  cmd_printf("mico_wlan_send_mgnt\r\n");

}

static void readRegister(char *pcWriteBuffer, int xWriteBufferLen,int argc, char **argv)
{
    Si115xReadFromRegister(atoi(argv[1]));
}

static void force(char *pcWriteBuffer, int xWriteBufferLen,int argc, char **argv)
{
    // Start next measurement
    Si115xForce();
}

static void start(char *pcWriteBuffer, int xWriteBufferLen,int argc, char **argv)
{
    // Start auto measurement
    Si115xStart();
}

static void hand(char *pcWriteBuffer, int xWriteBufferLen,int argc, char **argv)
{
    // Sensor data ready
    // Process measurement
    Si115xHandler();
}

static void hostout(char *pcWriteBuffer, int xWriteBufferLen,int argc, char **argv)
{
    // Start next measurement
    getSensorDataByHostout();
}

struct cli_command built_user[] = {
  //{"probe", "trigger 80211 probe req", probe_request_command},
  {"readreg", "read register", readRegister},

  {"force", "force", force},
  {"start", "start", start},
  {"hand", "hand", hand},
  {"host", "by host out", hostout},
};

OSStatus opple_cli_register( void )
{
    /* add our built-in commands */
    if( 0 == cli_register_commands(built_user, sizeof(built_user)/sizeof(struct cli_command)))
        return kNoErr;
    else
        return kGeneralErr;
}
