#include "../inc/mqtt.h"

#define MQTT_BROKER_URI "YOUR_URI"
#define MQTT_TOPIC "stm32/telemetry"

static const char *TAG_MQTT = "MQTT";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "Polaczono z brokerem MQTT!");
            mqtt_connected = true;
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG_MQTT, "Rozlaczono z brokerem MQTT!");
            mqtt_connected = false;
            break;

        default:
            break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

bool mqtt_publish_telemetry(const char *json_payload)
{
    if (!mqtt_connected || !mqtt_client) return false;
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_payload, 0, 1, 1);
    return (msg_id != -1);
}