/*
 * LD14P raw UART -> TCP Wi-Fi bridge for ESP32-S3 #2
 * UART1 RX: GPIO16, 8N1, configurable baud (115200 by default for this robot).
 * Does NOT decode LiDAR, talk micro-ROS, touch ESP32 #1 or send data on UART0.
 * TCP server on configurable port, exactly one active client; raw binary bytes only.
 * Target: ESP-IDF 5.x, ESP32-S3; educational robot, not safety rated.
 */
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define LIDAR_PORT_UART     UART_NUM_1
#define LIDAR_GPIO_RX      16
#define UART_RX_RING_BYTES (16 * 1024)
#define READ_BYTES         512
#define WIFI_UP_BIT        BIT0

static const char *TAG = "LD14_WIFI";
static EventGroupHandle_t s_wifi_events;

static void wifi_event_handler(void *ctx, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)ctx;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_UP_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = (const ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WIFI READY. ESP32 IP=" IPSTR " TCP PORT=%d",
                 IP2STR(&ev->ip_info.ip), CONFIG_LIDAR_TCP_PORT);
        xEventGroupSetBits(s_wifi_events, WIFI_UP_BIT);
    }
}

static void init_wifi(void)
{
    const size_t ssid_len = strlen(CONFIG_LIDAR_WIFI_SSID);
    const size_t pass_len = strlen(CONFIG_LIDAR_WIFI_PASSWORD);
    if (!ssid_len || ssid_len > 32 || pass_len > 63) {
        ESP_LOGE(TAG, "Set valid SSID/password via idf.py menuconfig first");
        abort();
    }

    s_wifi_events = xEventGroupCreate();
    configASSERT(s_wifi_events != NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));
    wifi_config_t net = {0};
    memcpy(net.sta.ssid, CONFIG_LIDAR_WIFI_SSID, ssid_len);
    memcpy(net.sta.password, CONFIG_LIDAR_WIFI_PASSWORD, pass_len);
    /* Auth mode is negotiated with the AP (supports open networks for lab use).
       Use WPA2/WPA3 on a trusted local network in normal operation. */
    net.sta.threshold.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &net));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

static void init_lidar_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = CONFIG_LIDAR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(LIDAR_PORT_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_PORT_UART, UART_PIN_NO_CHANGE,
                                  LIDAR_GPIO_RX, UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_PORT_UART, UART_RX_RING_BYTES,
                                        0, 0, NULL, 0));
    ESP_LOGI(TAG, "UART1 RX GPIO%d, baud %d, 8N1", LIDAR_GPIO_RX,
             CONFIG_LIDAR_UART_BAUD);
}

/* Return false on network failure or socket backpressure; reconnect cleanly.
 * SO_SNDTIMEO bounds the time UART is not read while a client is stalled. */
static bool send_all(int fd, const uint8_t *buf, size_t bytes)
{
    size_t offset = 0;
    while (offset < bytes) {
        ssize_t n = send(fd, buf + offset, bytes - offset, 0);
        if (n > 0) {
            offset += (size_t)n;
        } else if (n < 0 && errno == EINTR) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

static void client_stream(int fd)
{
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 250000};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    uint8_t chunk[READ_BYTES];
    uint64_t forwarded = 0;
    TickType_t last_report = xTaskGetTickCount();

    /* Drop stale UART bytes accumulated while disconnected. The ROS driver
     * resynchronizes at the next valid packet header and CRC. */
    ESP_ERROR_CHECK(uart_flush_input(LIDAR_PORT_UART));
    ESP_LOGI(TAG, "TCP client connected; streaming raw LD14P UART bytes");

    while (xEventGroupGetBits(s_wifi_events) & WIFI_UP_BIT) {
        /* Socket closed? Check non-blockingly even if lidar is unplugged. */
        uint8_t discard;
        ssize_t peek = recv(fd, &discard, 1, MSG_PEEK | MSG_DONTWAIT);
        if (peek == 0) break;
        if (peek < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            break;
        if (peek > 0) {
            /* This bridge is one-way; commands are intentionally ignored.
             * Consume unexpected data to avoid a permanently readable socket. */
            (void)recv(fd, &discard, 1, MSG_DONTWAIT);
        }

        int n = uart_read_bytes(LIDAR_PORT_UART, chunk, sizeof(chunk),
                                pdMS_TO_TICKS(20));
        if (n > 0) {
            if (!send_all(fd, chunk, (size_t)n)) {
                ESP_LOGW(TAG, "TCP send stalled or failed: errno=%d", errno);
                break;
            }
            forwarded += (uint32_t)n;
        }
        if (xTaskGetTickCount() - last_report > pdMS_TO_TICKS(10000)) {
            ESP_LOGI(TAG, "forwarded=%llu bytes (cumulative)",
                     (unsigned long long)forwarded);
            last_report = xTaskGetTickCount();
        }
    }
    shutdown(fd, SHUT_RDWR);
    close(fd);
    ESP_LOGW(TAG, "TCP client disconnected. Waiting for reconnection.");
}

static void tcp_bridge_task(void *ctx)
{
    (void)ctx;
    for (;;) {
        xEventGroupWaitBits(s_wifi_events, WIFI_UP_BIT, pdFALSE,
                            pdTRUE, portMAX_DELAY);
        int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_fd < 0) {
            ESP_LOGE(TAG, "socket() failed errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        int one = 1;
        setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        struct sockaddr_in addr = {
            .sin_family = AF_INET,
            .sin_port = htons(CONFIG_LIDAR_TCP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };
        if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
            listen(listen_fd, 1) != 0) {
            ESP_LOGE(TAG, "bind/listen failed errno=%d", errno);
            close(listen_fd);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "TCP listening on port %d", CONFIG_LIDAR_TCP_PORT);

        while (xEventGroupGetBits(s_wifi_events) & WIFI_UP_BIT) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(listen_fd, &rfds);
            struct timeval timeout = {.tv_sec = 0, .tv_usec = 250000};
            int ready = select(listen_fd + 1, &rfds, NULL, NULL, &timeout);
            if (ready == 0 || (ready < 0 && errno == EINTR)) continue;
            if (ready < 0) {
                ESP_LOGW(TAG, "select() failed errno=%d", errno);
                break;
            }
            int client_fd = accept(listen_fd, NULL, NULL);
            if (client_fd < 0) continue;
            if (xEventGroupGetBits(s_wifi_events) & WIFI_UP_BIT)
                client_stream(client_fd);
            else
                close(client_fd);
        }
        close(listen_fd);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    init_lidar_uart();
    init_wifi();
    BaseType_t created = xTaskCreate(tcp_bridge_task, "lidar_tcp_bridge",
                                     6144, NULL, 5, NULL);
    configASSERT(created == pdPASS);
    ESP_LOGI(TAG, "ESP32-S3 #2 LD14P Wi-Fi bridge ready (raw TCP, NOT micro-ROS)");
}
