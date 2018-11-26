#ifndef _WIFI_TCP_SERVER_STA_H_
#define	_WIFI_TCP_SERVER_STA_H_

#include <stdio.h>
#include <string.h>
#include <sys/fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "nvs_flash.h"

/* DEFINES */
#define SSID "Livebox-23C6"
#define PASSPHARSE "9E9C696156C2672FC24FA299DD"

#define MESSAGE "Hello TCP Client!!"
#define LISTENQ 2

enum {SOCKET_COMMAND_NONE = 0x00, \
        SOCKET_COMMAND_PRINT_IP_ADDRESS = 0x01, \
        SOCKET_COMMAND_SCAN_I2C = 0x02, \
        SOCKET_COMMAND_READ_OV7670 = 0x03};

/* GLOBAL VARIABLES */
EventGroupHandle_t wifi_event_group;
int socket_command_received;

/* FUNCTIONS */
void wifi_initialise();
void wifi_network_connect();
esp_err_t wifi_event_handler(void *ctx, system_event_t *event);
void wifi_print_ip_task(void *pvParam);
void wifi_print_ip_address();
void tcp_server_task(void *pvParam);
int get_socket_command();

#endif