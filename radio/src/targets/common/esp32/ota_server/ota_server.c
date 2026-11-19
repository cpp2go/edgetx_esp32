/*
 * Copyright (C) OpenTX / EdgeTX
 *
 * Based on code named
 *   https://github.com/yanbe/esp32-ota-server
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lwip/sockets.h"

#include "ota_server.h"

static const char *TAG = "OTA";

static int connect_socket = -1;
static int server_socket = -1;
static SemaphoreHandle_t ota_mutex = NULL;
static volatile enum OtaState otaState = OTA_IDLE;

enum OtaState ota_server_get_state(void)
{
    return otaState;
}

static void set_state(enum OtaState st)
{
    if (ota_mutex) {
        xSemaphoreTake(ota_mutex, portMAX_DELAY);
        otaState = st;
        xSemaphoreGive(ota_mutex);
    } else {
        otaState = st;
    }
}

static enum OtaState get_state(void)
{
    enum OtaState st;
    if (ota_mutex) {
        xSemaphoreTake(ota_mutex, portMAX_DELAY);
        st = otaState;
        xSemaphoreGive(ota_mutex);
    } else {
        st = otaState;
    }
    return st;
}

static void close_fd(int *fd)
{
    if (fd && *fd >= 0) {
        shutdown(*fd, SHUT_RDWR);
        close(*fd);
        *fd = -1;
    }
}

static void close_all_sockets(void)
{
    close_fd(&connect_socket);
    close_fd(&server_socket);
}

static int parse_content_length(const char *hdr, size_t hdr_len, int *out_len)
{
    const char *key = "Content-Length:";
    const size_t key_len = strlen(key);
    for (size_t i = 0; i + key_len < hdr_len; i++) {
        if (strncasecmp(&hdr[i], key, key_len) != 0) {
            continue;
        }
        const char *p = &hdr[i + key_len];
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        char *end = NULL;
        long v = strtol(p, &end, 10);
        if (end == p || v <= 0 || v > (16L * 1024L * 1024L)) {
            return -1;
        }
        *out_len = (int)v;
        return 0;
    }
    return -1;
}

static esp_err_t create_tcp_server(void)
{
    ESP_LOGI(TAG, "listening on port %d", OTA_LISTEN_PORT);

    server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_socket < 0) {
        ESP_LOGE(TAG, "socket create failed: errno=%d", errno);
        return ESP_FAIL;
    }

    int opt = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(OTA_LISTEN_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind failed: errno=%d", errno);
        close_all_sockets();
        return ESP_FAIL;
    }
    if (listen(server_socket, 1) < 0) {
        ESP_LOGE(TAG, "listen failed: errno=%d", errno);
        close_all_sockets();
        return ESP_FAIL;
    }

    // Interruptible accept: wake periodically so stop() can close the socket.
    while (get_state() == OTA_WAITING) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(server_socket, &rfds);
        struct timeval tv = {.tv_sec = 0, .tv_usec = 200 * 1000};
        int sel = select(server_socket + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) {
            if (errno == EINTR) {
                continue;
            }
            ESP_LOGE(TAG, "select failed: errno=%d", errno);
            close_all_sockets();
            return ESP_FAIL;
        }
        if (sel == 0) {
            continue;
        }

        struct sockaddr_in client_addr;
        socklen_t socklen = sizeof(client_addr);
        connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
        if (connect_socket < 0) {
            if (get_state() != OTA_WAITING) {
                close_all_sockets();
                return ESP_ERR_INVALID_STATE;
            }
            ESP_LOGE(TAG, "accept failed: errno=%d", errno);
            close_all_sockets();
            return ESP_FAIL;
        }

        // Stop listening for further clients while this update runs.
        close_fd(&server_socket);

        struct timeval rcv_to = {.tv_sec = 30, .tv_usec = 0};
        setsockopt(connect_socket, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));
        ESP_LOGI(TAG, "client connected");
        return ESP_OK;
    }

    close_all_sockets();
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t recv_http_headers(char *buf, size_t buf_size, size_t *hdr_len,
                                   size_t *body_in_buf)
{
    size_t total = 0;
    *hdr_len = 0;
    *body_in_buf = 0;

    while (total + 1 < buf_size) {
        if (get_state() == OTA_STOPPING) {
            return ESP_ERR_INVALID_STATE;
        }
        int n = recv(connect_socket, buf + total, buf_size - 1 - total, 0);
        if (n == 0) {
            ESP_LOGE(TAG, "peer closed before headers complete");
            return ESP_FAIL;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                ESP_LOGE(TAG, "recv headers timed out");
            } else {
                ESP_LOGE(TAG, "recv headers failed: errno=%d", errno);
            }
            return ESP_FAIL;
        }
        total += (size_t)n;
        buf[total] = '\0';

        char *end = strstr(buf, "\r\n\r\n");
        if (end) {
            *hdr_len = (size_t)(end - buf) + 4;
            *body_in_buf = total - *hdr_len;
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "HTTP headers too large");
    return ESP_FAIL;
}

void ota_server_start(void)
{
    if (ota_mutex == NULL) {
        ota_mutex = xSemaphoreCreateMutex();
        if (ota_mutex == NULL) {
            ESP_LOGE(TAG, "mutex create failed");
            return;
        }
    }

    set_state(OTA_WAITING);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "no OTA partition (need ota_0/ota_1 + otadata in partition table)");
        set_state(OTA_IDLE);
        return;
    }

    esp_err_t err = create_tcp_server();
    if (err != ESP_OK) {
        set_state(OTA_IDLE);
        return;
    }

    set_state(OTA_UPDATING);
    ESP_LOGI(TAG, "writing to '%s' subtype=%d offset=0x%lx", update_partition->label,
             update_partition->subtype, (unsigned long)update_partition->address);

    char *ota_buff = (char *)malloc(OTA_BUFF_SIZE);
    if (!ota_buff) {
        ESP_LOGE(TAG, "OOM for OTA buffer");
        close_all_sockets();
        set_state(OTA_IDLE);
        return;
    }

    size_t hdr_len = 0;
    size_t body_in_buf = 0;
    err = recv_http_headers(ota_buff, OTA_BUFF_SIZE, &hdr_len, &body_in_buf);
    if (err != ESP_OK) {
        free(ota_buff);
        close_all_sockets();
        set_state(OTA_IDLE);
        return;
    }

    int content_length = -1;
    if (parse_content_length(ota_buff, hdr_len, &content_length) != 0) {
        ESP_LOGE(TAG, "missing/invalid Content-Length");
        const char *resp = "HTTP/1.0 400 Bad Request\r\n\r\nMissing Content-Length\n";
        send(connect_socket, resp, strlen(resp), 0);
        free(ota_buff);
        close_all_sockets();
        set_state(OTA_IDLE);
        return;
    }
    ESP_LOGI(TAG, "Content-Length=%d", content_length);

    if ((size_t)content_length > update_partition->size) {
        ESP_LOGE(TAG, "image too large (%d > %lu)", content_length,
                 (unsigned long)update_partition->size);
        const char *resp = "HTTP/1.0 400 Bad Request\r\n\r\nImage too large\n";
        send(connect_socket, resp, strlen(resp), 0);
        free(ota_buff);
        close_all_sockets();
        set_state(OTA_IDLE);
        return;
    }

    esp_ota_handle_t ota_handle = 0;
    err = esp_ota_begin(update_partition, content_length, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        const char *resp = "HTTP/1.0 500 Internal Server Error\r\n\r\nota_begin failed\n";
        send(connect_socket, resp, strlen(resp), 0);
        free(ota_buff);
        close_all_sockets();
        set_state(OTA_IDLE);
        return;
    }

    int content_received = 0;
    if (body_in_buf > 0) {
        err = esp_ota_write(ota_handle, ota_buff + hdr_len, body_in_buf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write(header body): %s", esp_err_to_name(err));
            goto ota_fail;
        }
        content_received += (int)body_in_buf;
    }

    while (content_received < content_length) {
        if (get_state() == OTA_STOPPING) {
            err = ESP_ERR_INVALID_STATE;
            goto ota_fail;
        }
        int remain = content_length - content_received;
        int chunk = remain > (OTA_BUFF_SIZE - 1) ? (OTA_BUFF_SIZE - 1) : remain;
        int n = recv(connect_socket, ota_buff, chunk, 0);
        if (n == 0) {
            ESP_LOGE(TAG, "peer closed early (%d/%d)", content_received, content_length);
            err = ESP_FAIL;
            goto ota_fail;
        }
        if (n < 0) {
            ESP_LOGE(TAG, "recv body failed: errno=%d (%d/%d)", errno, content_received,
                     content_length);
            err = ESP_FAIL;
            goto ota_fail;
        }
        err = esp_ota_write(ota_handle, ota_buff, n);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            goto ota_fail;
        }
        content_received += n;
    }

    err = esp_ota_end(ota_handle);
    ota_handle = 0;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        goto ota_fail_no_abort;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        goto ota_fail_no_abort;
    }

    {
        char res_buff[128];
        int send_len = snprintf(res_buff, sizeof(res_buff),
                                "HTTP/1.0 200 OK\r\n\r\nSuccess. Next boot partition is %s\n",
                                update_partition->label);
        if (send_len > 0) {
            send(connect_socket, res_buff, send_len, 0);
        }
    }

    ESP_LOGI(TAG, "OTA ok, %d bytes written to %s — restarting", content_received,
             update_partition->label);
    free(ota_buff);
    close_all_sockets();
    set_state(OTA_IDLE);
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
    return;

ota_fail:
    if (ota_handle) {
        esp_ota_abort(ota_handle);
        ota_handle = 0;
    }
ota_fail_no_abort:
{
    char res_buff[96];
    int send_len = snprintf(res_buff, sizeof(res_buff),
                            "HTTP/1.0 400 Bad Request\r\n\r\nFailure. Error: %s\n",
                            esp_err_to_name(err));
    if (send_len > 0 && connect_socket >= 0) {
        send(connect_socket, res_buff, send_len, 0);
    }
}
    free(ota_buff);
    close_all_sockets();
    set_state(OTA_IDLE);
}

void ota_server_stop(void)
{
    if (!ota_mutex) {
        return;
    }

    for (;;) {
        xSemaphoreTake(ota_mutex, portMAX_DELAY);
        enum OtaState st = otaState;
        if (st == OTA_IDLE) {
            xSemaphoreGive(ota_mutex);
            return;
        }
        if (st == OTA_UPDATING) {
            // Do not tear down mid-flash; wait until finish/fail/restart.
            xSemaphoreGive(ota_mutex);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // WAITING / already STOPPING: close fds so select/accept wakes.
        otaState = OTA_STOPPING;
        int srv = server_socket;
        int cli = connect_socket;
        server_socket = -1;
        connect_socket = -1;
        xSemaphoreGive(ota_mutex);

        if (srv >= 0) {
            shutdown(srv, SHUT_RDWR);
            close(srv);
        }
        if (cli >= 0) {
            shutdown(cli, SHUT_RDWR);
            close(cli);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        if (get_state() == OTA_IDLE) {
            return;
        }
        // Orphaned STOPPING (start already exited): force idle.
        if (get_state() == OTA_STOPPING) {
            vTaskDelay(pdMS_TO_TICKS(200));
            if (get_state() == OTA_STOPPING) {
                set_state(OTA_IDLE);
            }
            return;
        }
    }
}
