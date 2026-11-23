#ifndef MQTT_H
#define MQTT_H

# include <stdbool.h>
# include "mqtt_client.h"
# include "esp_log.h"

void mqtt_app_start(void);
bool mqtt_publish_telemetry(const char *json_payload);

#endif