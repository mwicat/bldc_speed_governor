#include <stdio.h>
#include <stdlib.h> 

#include <Arduino.h>

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

#include <pidreg.h>


// FG
const gpio_num_t pulse_gpio_num = GPIO_NUM_23;
const int pulseCnt = 5;
const int pulsePerRev = 4;

// PWM
const uint8_t pwm_channel = 0;
const gpio_num_t pwm_gpio_num = GPIO_NUM_19;

const uint32_t pwm_frequency = 25000;
const uint8_t pwm_resolution = 11;

const uint32_t pwm_duty_min = 200;
const uint32_t pwm_duty_max = 2047;

volatile uint32_t pwm_duty = 204;

// PCNT
volatile unsigned long fg_period = 0;
volatile double fg_frequency = 0;

unsigned long last_fg_pulse_millis = 0;
bool got_fg_pulse = false;


// PID Regulator
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t complexHandlerTask;

hw_timer_t * adcTimer = NULL;
double timer_freq = 10;

const gpio_num_t timer_flash_gpio_num = GPIO_NUM_17;

pidreg_state_t* pidreg_state;

volatile bool pidreg_enabled = false;

volatile pidreg_config_t pidreg_config = {
  .Kp = 0.1,
  .Ki = 0.1,
  .Kd = 0,
  .output_min = pwm_duty_min,
  .output_max = pwm_duty_max,
  .setpoint = 400
};


static void IRAM_ATTR pulse_counter_handler(void *arg) {
    uint32_t intr_status = PCNT.int_st.val;
    
    for (int i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            unsigned long current_millis = millis();
            if (got_fg_pulse) {
              fg_period = (current_millis - last_fg_pulse_millis) / pulseCnt;
              if (fg_period > 0) {
                fg_frequency = (1.0/fg_period) * 1000 * 60 / 4;
              }
            }
            last_fg_pulse_millis = current_millis;
            got_fg_pulse = true;

            PCNT.int_clr.val = BIT(i);
        }
    }
}

void setup_pulse_counter() {
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

  ESP_ERROR_CHECK(pcnt_isr_register(pulse_counter_handler, NULL, 0, NULL));

  ESP_ERROR_CHECK(pcnt_intr_enable(pcnt_config.unit));

  ESP_ERROR_CHECK(pcnt_counter_resume(pcnt_config.unit));  
}

void IRAM_ATTR onPIDRegulatorTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  gpio_set_level(timer_flash_gpio_num, 1);

  if (pidreg_enabled) {
    pidreg_config_t pidreg_config_copy = {
      .Kp = pidreg_config.Kp,
      .Ki = pidreg_config.Kp,
      .Kd = pidreg_config.Kp,
      .output_min = pidreg_config.output_min,
      .output_max = pidreg_config.output_max,
      .setpoint = pidreg_config.setpoint
    };

    pwm_duty = round(pidreg_compute(pidreg_state, &pidreg_config_copy, fg_frequency));

    ledcWrite(pwm_channel, pwm_duty);
  }

  gpio_set_level(timer_flash_gpio_num, 0);

  portEXIT_CRITICAL_ISR(&timerMux);
}

void setPIDRegulatorFrequency(double frequency) {
  int ticks = round(1.0/frequency * 1000000);
  timerAlarmWrite(adcTimer, ticks, true);
}

void setupPIDRegulator() {
  pidreg_state = pidreg_init(pwm_duty, fg_frequency);

  adcTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(adcTimer, &onPIDRegulatorTimer, true);
  setPIDRegulatorFrequency(timer_freq);
  timerAlarmEnable(adcTimer);
}

void setup() {
  Serial.begin(115200);

  gpio_set_direction(pulse_gpio_num, GPIO_MODE_INPUT);

  gpio_set_direction(pwm_gpio_num, GPIO_MODE_OUTPUT);

  gpio_set_direction(timer_flash_gpio_num, GPIO_MODE_OUTPUT);


  ledcSetup(pwm_channel, pwm_frequency, pwm_resolution);
  ledcAttachPin(pwm_gpio_num, pwm_channel);
  ledcWrite(pwm_channel, pwm_duty);

  setup_pulse_counter();

  setupPIDRegulator();
}

bool readLineSerial(char* buf, size_t length) {
  size_t read_sz = 0;
  bool got_input = (Serial.available() > 0);

  if (got_input) {
    read_sz = Serial.readBytesUntil('\n', buf,  length);
    if (read_sz > 0 && buf[read_sz-1] == '\r') read_sz--;
    buf[read_sz] = '\0';
  }
  return got_input;
}


extern "C" void app_main() {
    setup();

    char line_buf[255];

    for(;;) {
      bool got_input = readLineSerial(line_buf, sizeof(line_buf));
      if (got_input) {
        char cmd;
        double val;
        if (sscanf(line_buf, "%c %lf", &cmd, &val) != 2) {
          Serial.printf("Error parsing input '%s'\r\n", line_buf);
        }
        switch (cmd) {
          case 'a':
            pwm_duty = val;
            ledcWrite(pwm_channel, pwm_duty);
            Serial.print("ok\r\n");
            break;
          case 'e':
            pidreg_enabled = (val == 0) ? 0 : 1;
            Serial.printf("ok %d\r\n", pidreg_enabled);
            break;
          case 's':
            pidreg_config.setpoint = val;
            Serial.print("ok\r\n");
            break;
          case 'p':
            pidreg_config.Kp = val;
            Serial.print("ok\r\n");
            break;
          case 'i':
            pidreg_config.Ki = val;
            Serial.print("ok\r\n");
            break;
          case 'd':
            pidreg_config.Kd = val;
            Serial.print("ok\r\n");
            break;
          case 'f':
            setPIDRegulatorFrequency(val);
            break;
          case 'r':
            Serial.printf("r a=%d,e=%d,s=%lf,p=%lf,i=%lf,d=%lf,f=%lf\r\n",
              pwm_duty,
              pidreg_enabled,
              pidreg_config.setpoint,
              pidreg_config.Kp,
              pidreg_config.Ki,
              pidreg_config.Kd,
              timer_freq
              );
            break;
        }
      }

      vTaskDelay(pdMS_TO_TICKS(500));
    }  
}