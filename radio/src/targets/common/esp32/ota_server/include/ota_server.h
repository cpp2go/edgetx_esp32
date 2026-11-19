/*
 * Copyright (C) OpenTX / EdgeTX
 *
 * Based on code named
 *   https://github.com/yanbe/esp32-ota-server
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

#ifndef _OTA_SERVER_H_
#define _OTA_SERVER_H_

#define OTA_LISTEN_PORT 8032
#define OTA_BUFF_SIZE   4096

enum OtaState {
    OTA_IDLE = 0,
    OTA_WAITING,
    OTA_UPDATING,
    OTA_STOPPING
};

#ifdef __cplusplus
extern "C" {
#endif

void ota_server_start(void);
void ota_server_stop(void);
enum OtaState ota_server_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* _OTA_SERVER_H_ */
