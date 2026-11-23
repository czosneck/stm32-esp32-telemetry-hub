#ifndef WIFI_STA_H
#define WIFI_STA_H

# include "nvs_flash.h"
# include "freertos/FreeRTOS.h"
# include "freertos/event_groups.h"
# include "esp_system.h"
# include "esp_wifi.h"
# include "esp_event.h"
# include "esp_log.h"

void wifi_init_sta(void);

#endif