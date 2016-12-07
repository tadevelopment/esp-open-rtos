/*
 * Derived from examples/mqtt_client/mqtt_client.c - added TLS1.2 support and some minor modifications.
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>
#include <fastmath.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

// this must be ahead of any mbedtls header files so the local mbedtls/config.h can be properly referenced
#include "ssl_connection.h"

#define MQTT_PUB_TOPIC "eq/jim/msgs"
#define MQTT_SUB_TOPIC "eq/jim/ctrl"
#define GPIO_LED 2
#define MQTT_PORT 8883

/* certs, key, and endpoint */
extern char *ca_cert, *client_endpoint, *client_cert, *client_key;

static int wifi_alive = 0;
static int ssl_reset;
static SSLConnection *ssl_conn;
static QueueHandle_t publish_queue;

//////////////////////////////////////////////////////////////
typedef struct Timeout {
    int state_cntr;
    uint16_t last_time;
} Timeout;

void timeout_init(Timeout *cntr, uint16_t timeout_ms)
{
    cntr->state_cntr = timeout_ms / portTICK_PERIOD_MS;
    cntr->last_time = xTaskGetTickCount();
}
bool timeout_expired(Timeout *cntr)
{
    uint16_t sampleT = xTaskGetTickCount();
    if (sampleT > cntr->last_time)
    {
        uint16_t offset = sampleT - cntr->last_time;
        cntr->state_cntr -= offset;
    }
    if (sampleT < cntr->last_time) // In the case of a loop
    {
        // This else block should never really execute, however
        // its here in the incredibly unlikely event that sampleT
        // is never greater than lastT.  In that case, we 
        // will still eventually exit this loop
        cntr->state_cntr -= 1;
    }
    cntr->last_time = sampleT;

    // return TRUE if this counter has expired
    return cntr->state_cntr < 0;
}

//////////////////////////////////////////////////////////////

// ADC resolution is 10 bits
#define ADC_BITS  10 
#define ADC_RES (1<<ADC_BITS)
#define ADC_MIDPOINT (ADC_RES/2)

// Whats the period (in seconds) that we publish values for?
#define PUBLISH_PERIOD_SEC  10
#define PUBLISH_PERIOD_MS (PUBLISH_PERIOD_SEC * 1000)

// Magic calibration number (taken from openenergymonitor
#define ICAL 111.1
#define I_RATIO (ICAL * ADC_RES)

bool delay_till_adc_wave_crossing(uint16_t timeout_ms)
{
    uint16_t sampleI;

    // We use a timeout to ensure we don't wait forever
    // for a crossing (if the input is v. dirty, or some other error)
    Timeout timeout;
    timeout_init(&timeout, timeout_ms);

    while(1)
    {
        if (!wifi_alive)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        sampleI = sdk_system_adc_read();
        if ((sampleI < (ADC_RES*0.55)) && (sampleI > (ADC_RES*0.45)))
            return true; // Midpoint found

        // Count down the time used
        if (timeout_expired(&timeout) == true)
            return false; 
    }
}

void publish_avg_sample(double avg_sample)
{
    static double Irms;
    Irms = avg_sample;
    //storage = avg_sample;

    //double I_RATIO = ICAL * ADC_RES;
    //Irms = I_RATIO * sqrt(avg_sample);

    printf("Publishing Irms\r\n");

    if (xQueueSend(publish_queue, (void *) &Irms, 0) == pdFALSE) {
        printf("Publish queue overflow\r\n");
    }
}

static void accumulate_adc_task(void *pvParameters) 
{
    // Try to find a start point at about the midpoint
    // of the phase (where the adc reads close to MIDPOINT)
    bool on_crossing = delay_till_adc_wave_crossing(3000);
    if (!on_crossing)
        printf("ERROR: No crossing value found when initializing ADC task\n");

    uint16_t sampleI;
    double offsetI = ADC_MIDPOINT;
    double filteredI;
    double sqI;
    double sumI = 0;

    uint32_t num_samples = 0;

    // Now, we accumulate values forever.  
    // However after a certain amount of time
    // we push values to be published.
    Timeout timeout;
    timeout_init(&timeout, PUBLISH_PERIOD_MS);
    while (1)
    {
        sampleI = sdk_system_adc_read();

        // Digital low pass filter extracts the 0.5V dc offset,
        //  then subtract this - signal is now centered on 0 counts.
        offsetI = (offsetI + (sampleI-offsetI)/1024);
        filteredI = sampleI - offsetI;

        // Root-mean-square method current
        // 1) square current values
        sqI = filteredI * filteredI;
        // 2) sum
        sumI += sqI;
        num_samples++;

        if (timeout_expired(&timeout) && wifi_alive)
        {
            double avg_sample = sumI / num_samples;
            printf("Publishing avg: %f, sum: %f, samples: %d\n\r", avg_sample, sumI, num_samples);
            publish_avg_sample(avg_sample);

           
            // Initialize back 0, we begin a new recording period.
            num_samples = 0;
            sumI = 0;
            timeout_init(&timeout, PUBLISH_PERIOD_MS);
        }
        else
        {
            // We sample as often as possible, but still need to 
            // give the other threads opportunity to run
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
}

// double calcIrms(unsigned int Number_of_Samples)
// {
//     int SupplyVoltage=1000;

//     uint16_t sampleI;
//     for (unsigned int n = 0; n < Number_of_Samples; n++)
//     {
//         sampleI = sdk_system_adc_read();

//         // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
//         //  then subtract this - signal is now centered on 0 counts.
//         offsetI = (offsetI + (sampleI-offsetI)/1024);
//         filteredI = sampleI - offsetI;

//         // Root-mean-square method current
//         // 1) square current values
//         sqI = filteredI * filteredI;
//         // 2) sum
//         sumI += sqI;
//     }

//     double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
//     Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

//     //Reset accumulators
//     sumI = 0;
//     //--------------------------------------------------------------------------------------

// return Irms;
// }

//////////////////////////////////////////////////////////////
// static void beat_task(void *pvParameters) {
//     char msg[16];
//     int count = 0;

//     while (1) {
//         if (!wifi_alive) {
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//             continue;
//         }

//         printf("Schedule to publish\r\n");

//         snprintf(msg, sizeof(msg), "%d", count);
//         if (xQueueSend(publish_queue, (void *) msg, 0) == pdFALSE) {
//             printf("Publish queue overflow\r\n");
//         }

//         vTaskDelay(10000 / portTICK_PERIOD_MS);
//     }
// }

static void topic_received(mqtt_message_data_t *md) {
    mqtt_message_t *message = md->message;
    int i;

    printf("Received: ");
    for (i = 0; i < md->topic->lenstring.len; ++i)
        printf("%c", md->topic->lenstring.data[i]);

    printf(" = ");
    for (i = 0; i < (int) message->payloadlen; ++i)
        printf("%c", ((char *) (message->payload))[i]);
    printf("\r\n");

    if (!strncmp(message->payload, "on", 2)) {
        printf("Turning on LED\r\n");
        gpio_write(GPIO_LED, 0);
    } else if (!strncmp(message->payload, "off", 3)) {
        printf("Turning off LED\r\n");
        gpio_write(GPIO_LED, 1);
    }
}

static const char *get_my_id(void) {
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *) my_id))
        return NULL;
    for (i = 5; i >= 0; --i) {
        x = my_id[i] & 0x0F;
        if (x > 9)
            x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9)
            x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

static int mqtt_ssl_read(mqtt_network_t * n, unsigned char* buffer, int len,
        int timeout_ms) {
    int r = ssl_read(ssl_conn, buffer, len, timeout_ms);
    if (r <= 0
            && (r != MBEDTLS_ERR_SSL_WANT_READ
                    && r != MBEDTLS_ERR_SSL_WANT_WRITE
                    && r != MBEDTLS_ERR_SSL_TIMEOUT)) {
        printf("%s: TLS read error (%d), resetting\n\r", __func__, r);
        ssl_reset = 1;
    };
    return r;
}

static int mqtt_ssl_write(mqtt_network_t* n, unsigned char* buffer, int len,
        int timeout_ms) {
    int r = ssl_write(ssl_conn, buffer, len, timeout_ms);
    if (r <= 0
            && (r != MBEDTLS_ERR_SSL_WANT_READ
                    && r != MBEDTLS_ERR_SSL_WANT_WRITE)) {
        printf("%s: TLS write error (%d), resetting\n\r", __func__, r);
        ssl_reset = 1;
    }
    return r;
}

static void mqtt_task(void *pvParameters) {
    int ret = 0;
    struct mqtt_network network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    ssl_conn = (SSLConnection *) malloc(sizeof(SSLConnection));
    while (1) {
        if (!wifi_alive) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        printf("%s: started\n\r", __func__);
        ssl_reset = 0;
        ssl_init(ssl_conn);
        ssl_conn->ca_cert_str = ca_cert;
        ssl_conn->client_cert_str = client_cert;
        ssl_conn->client_key_str = client_key;

        mqtt_network_new(&network);
        network.mqttread = mqtt_ssl_read;
        network.mqttwrite = mqtt_ssl_write;

        printf("%s: connecting to MQTT server %s ... ", __func__,
                client_endpoint);
        ret = ssl_connect(ssl_conn, client_endpoint, MQTT_PORT);

        if (ret) {
            printf("error: %d\n\r", ret);
            ssl_destroy(ssl_conn);
            continue;
        }
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf,
                100);

        data.willFlag = 0;
        data.MQTTVersion = 4;
        data.cleansession = 1;
        data.clientID.cstring = mqtt_client_id;
        data.username.cstring = NULL;
        data.password.cstring = NULL;
        data.keepAliveInterval = 1000;
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if (ret) {
            printf("error: %d\n\r", ret);
            ssl_destroy(ssl_conn);
            continue;
        }
        printf("done\r\n");
        mqtt_subscribe(&client, MQTT_SUB_TOPIC, MQTT_QOS1, topic_received);
        xQueueReset(publish_queue);

        while (wifi_alive && !ssl_reset) {
            double value;
            char msg[128];
            while (xQueueReceive(publish_queue, (void *) &value, 0) == pdTRUE) {
                //TickType_t task_tick = xTaskGetTickCount();
                //uint32_t free_heap = xPortGetFreeHeapSize();
                //uint32_t free_stack = uxTaskGetStackHighWaterMark(NULL);
                //snprintf(msg, sizeof(msg), "%u: free heap %u, free stack %u",
                 //       task_tick, free_heap, free_stack * 4);
                //printf("Publishing: %s\r\n", msg);
                snprintf(msg, sizeof(msg), "{ \"irms\" : %f }", value);

                mqtt_message_t message;
                message.payload = msg;
                message.payloadlen = strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, MQTT_PUB_TOPIC, &message);
                if (ret != MQTT_SUCCESS) {
                    printf("error while publishing message: %d\n", ret);
                    break;
                }
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n\r");
        ssl_destroy(ssl_conn);
    }
}

static void wifi_task(void *pvParameters) {
    uint8_t status = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = { .ssid = WIFI_SSID, .password =
            WIFI_PASS, };

    printf("%s: Connecting to WiFi\n\r", __func__);
    sdk_wifi_set_opmode (STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while (1) {
        wifi_alive = 0;

        while ((status != STATION_GOT_IP) && (retries)) {
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status);
            if (status == STATION_WRONG_PASSWORD) {
                printf("WiFi: wrong password\n\r");
                break;
            } else if (status == STATION_NO_AP_FOUND) {
                printf("WiFi: AP not found\n\r");
                break;
            } else if (status == STATION_CONNECT_FAIL) {
                printf("WiFi: connection failed\r\n");
                break;
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            --retries;
        }

        while ((status = sdk_wifi_station_get_connect_status())
                == STATION_GOT_IP) {
            if (wifi_alive == 0) {
                printf("WiFi: Connected\n\r");
                wifi_alive = 1;
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        wifi_alive = 0;
        printf("WiFi: disconnected\n\r");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void user_init(void) {
    uart_set_baud(0, 115200);
    printf("SDK version: %s, free heap %u\n", sdk_system_get_sdk_version(),
            xPortGetFreeHeapSize());

    gpio_enable(GPIO_LED, GPIO_OUTPUT);
    gpio_write(GPIO_LED, 1);

    publish_queue = xQueueCreate(3, 16);
    xTaskCreate(&wifi_task, "wifi_task", 256, NULL, 2, NULL);
    xTaskCreate(&accumulate_adc_task, "acc_adc_task", 256, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 2048, NULL, 2, NULL);
}
