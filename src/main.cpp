#include <Arduino.h>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/pcnt_struct.h"


const int pwmFanPin1 = 19;
const int pulseCnt = 10;

int lastStamp0 = 0;

xQueueHandle pcnt_evt_queue;

typedef struct {
  int unit;
  uint32_t status;
  unsigned long timeStamp;
} pcnt_evt_t;

static void IRAM_ATTR pcnt_intr_handler(void *arg) {
  unsigned long currentMillis = millis();
  uint32_t intr_status = PCNT.int_st.val;
  int i = 0;
  pcnt_evt_t evt;
  portBASE_TYPE HPTaskAwoken = pdFALSE;


  for (i = 0; i < PCNT_UNIT_MAX; i++) {
    if (intr_status & (BIT(i))) {
      evt.unit = i;
      evt.status = PCNT.status_unit[i].val;
      evt.timeStamp = currentMillis;
      PCNT.int_clr.val = BIT(i);
      xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
      if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
      }
    }
  }
}


void pcnt_init_channel(const pcnt_config_t *pcnt_config) {
  ESP_ERROR_CHECK(pcnt_unit_config(pcnt_config));

  ESP_ERROR_CHECK(pcnt_set_filter_value(pcnt_config->unit, 100));
  ESP_ERROR_CHECK(pcnt_filter_enable(pcnt_config->unit));

  ESP_ERROR_CHECK(pcnt_event_enable(pcnt_config->unit, PCNT_EVT_ZERO));

  ESP_ERROR_CHECK(pcnt_counter_pause(pcnt_config->unit));
  ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_config->unit));

  ESP_ERROR_CHECK(pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL));

  ESP_ERROR_CHECK(pcnt_intr_enable(pcnt_config->unit));

  ESP_ERROR_CHECK(pcnt_counter_resume(pcnt_config->unit));
}


int countRPM(int firstTime, int lastTime, int pulseTotal, int pulsePerRev) {
  int timeDelta = (lastTime - firstTime);  //lastTime - firstTime
  if (timeDelta <= 0) {                    // This means we've gotten something wrong
    return -1;
  }
  return (int)round((60000 * ((float)pulseTotal / pulsePerRev)) / timeDelta);
}


int getRPM() {
  pcnt_evt_t evt;
  portBASE_TYPE res = xQueueReceive(pcnt_evt_queue, &evt, pdMS_TO_TICKS(1000));
  int RPM0 = -1; 

  if (res == pdTRUE) {
    if (evt.unit == 0) {
      if (lastStamp0 == 0) {
        lastStamp0 = evt.timeStamp;
      }
      RPM0 = countRPM(lastStamp0, evt.timeStamp, pulseCnt, 4);

      if (RPM0 == -1) {
        return -1;
      }
      lastStamp0 = evt.timeStamp;
    }
  }
  return RPM0;
}


void setup() {
  Serial.begin(115200);

  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));

  pcnt_config_t pcnt_config = {};

  pcnt_config.pulse_gpio_num = pwmFanPin1;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.counter_h_lim = pulseCnt;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.channel = PCNT_CHANNEL_0;

  pcnt_init_channel(&pcnt_config);
}


void loop() {
    int rpm = getRPM();
    printf("RPM:%d\n", rpm);
}



extern "C" void app_main() {
    setup();
    for(;;) {
        loop();
    }
}