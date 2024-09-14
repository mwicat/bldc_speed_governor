#include "FGTachometer.h"

static void IRAM_ATTR pcnt_intr_handler(void *arg) {
  QueueHandle_t pcnt_evt_queue = (QueueHandle_t)arg;
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

int calculate_rpm(int firstTime, int lastTime, int pulseTotal, int pulsePerRev)
{
    int timeDelta = (lastTime - firstTime);
    if (timeDelta <= 0)
    {
        return -1;
    }
    return (int)round((60000 * ((float)pulseTotal / pulsePerRev)) / timeDelta);
}

FGTachometer::FGTachometer(int pulse_gpio_num, int pulseCnt, int pulsePerRev) {
  this->pulse_gpio_num = pulse_gpio_num;
  this->pulseCnt = pulseCnt;
  this->pulsePerRev = pulsePerRev;

  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
}

void FGTachometer::init() {
  pcnt_config_t pcnt_config = {};

  pcnt_config.pulse_gpio_num = pulse_gpio_num;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.counter_h_lim = pulseCnt;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.channel = PCNT_CHANNEL_0;

  ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

  ESP_ERROR_CHECK(pcnt_set_filter_value(pcnt_config.unit, 100));
  ESP_ERROR_CHECK(pcnt_filter_enable(pcnt_config.unit));

  ESP_ERROR_CHECK(pcnt_event_enable(pcnt_config.unit, PCNT_EVT_ZERO));

  ESP_ERROR_CHECK(pcnt_counter_pause(pcnt_config.unit));
  ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_config.unit));

  ESP_ERROR_CHECK(pcnt_isr_register(pcnt_intr_handler, pcnt_evt_queue, 0, NULL));

  ESP_ERROR_CHECK(pcnt_intr_enable(pcnt_config.unit));

  ESP_ERROR_CHECK(pcnt_counter_resume(pcnt_config.unit));
}


int FGTachometer::getRPM(int timeout_ms) {
  pcnt_evt_t evt;

  portBASE_TYPE res = xQueueReceive(pcnt_evt_queue, &evt, pdMS_TO_TICKS(timeout_ms));

  int RPM0 = -1; 

  if (res == pdTRUE) {
    if (evt.unit == 0) {
      if (lastStamp0 == 0) {
        lastStamp0 = evt.timeStamp;
      }
      RPM0 = calculate_rpm(lastStamp0, evt.timeStamp, pulseCnt, pulsePerRev);

      if (RPM0 == -1) {
        return -1;
      }
      lastStamp0 = evt.timeStamp;
    }
  }
  return RPM0;
}