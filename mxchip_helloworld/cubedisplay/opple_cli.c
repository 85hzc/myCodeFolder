/*
 * opple_cli.c
 *
 *  Created on: 2019Äê2ÔÂ15ÈÕ
 *      Author: OP056666
 */

#include "mico.h"

static void probe_request_command(char *pcWriteBuffer, int xWriteBufferLen,int argc, char **argv)
{
#if 0
    mico_wlan_start_monitor
    mico_wlan_stop_monitor
    mico_wlan_monitor_set_channel
    mxchip_active_scan
    mico_wlan_register_monitor_cb(APPeerBeaconAction)
    wlan_inject_frame(const uint8_t *buff, size_t len)
    mico_wlan_send_mgnt(uint8_t *buffer, uint32_t length)

    mico_wlan_monitor_rx_type(WLAN_RX_ALL);
#endif

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

struct cli_command built_user[] = {
  {"probe", "trigger 80211 probe req", probe_request_command},

};

OSStatus opple_cli_register( void )
{
    /* add our built-in commands */
    if( 0 == cli_register_commands(built_user, sizeof(built_user)/sizeof(struct cli_command)))
        return kNoErr;
    else
        return kGeneralErr;
}
