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


#define LEDC_TIMER0_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER0_DUTY_RES LEDC_TIMER_11_BIT
#define LEDC_TIMER0_FREQ (25000)
#define LEDC_TIMER0_CLK LEDC_USE_APB_CLK

#define LEDC_TIMER0_DUTY_MAX (2048)

#define LEDC_CTL_GPIO GPIO_NUM_19
#define LEDC_CTL_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_CTL_CHANNEL LEDC_CHANNEL_0
#define LEDC_CTL_INTR LEDC_INTR_DISABLE
#define LEDC_CTL_TIMER LEDC_TIMER_0

#define PID_KP_DEFAULT (0.1)
#define PID_KI_DEFAULT (0.1)
#define PID_KD_DEFAULT (0)
#define PID_SETPOINT_DEFAULT (400)

#define PID_TIMING_FLAG_GPIO GPIO_NUM_17

#define LEDC_CTL_DUTY_MIN (200)
#define LEDC_CTL_DUTY_MAX LEDC_TIMER0_DUTY_MAX
#define LEDC_CTL_DUTY_DEFAULT LEDC_CTL_DUTY_MIN

#define LEDC_PID_KI_GPIO GPIO_NUM_16
#define LEDC_PID_KI_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_PID_KI_CHANNEL LEDC_CHANNEL_1
#define LEDC_PID_KI_INTR LEDC_INTR_DISABLE
#define LEDC_PID_KI_TIMER LEDC_TIMER_0

#define PCNT_GPIO GPIO_NUM_23
#define PCNT_COUNT_LIMIT (5)

#define FG_PULSE_PER_REV (4)

// PWM
volatile uint32_t ctl_pwm_duty = LEDC_CTL_DUTY_DEFAULT;

// PCNT
volatile unsigned long fg_period = 0;
volatile double fg_frequency = 0;

unsigned long last_fg_pulse_millis = 0;
bool got_fg_pulse = false;

// PID Regulator
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t complexHandlerTask;

hw_timer_t * timer_pid = NULL;
double timer_freq = 10;

pidreg_state_t* pidreg_state;

volatile bool pidreg_enabled = false;

volatile pidreg_config_t pidreg_config = {
  .Kp = PID_KP_DEFAULT,
  .Ki = PID_KI_DEFAULT,
  .Kd = PID_KD_DEFAULT,
  .output_min = LEDC_CTL_DUTY_MIN,
  .output_max = LEDC_CTL_DUTY_MAX,
  .setpoint = PID_SETPOINT_DEFAULT
};

static void IRAM_ATTR pulse_counter_handler(void *arg) {
    uint32_t intr_status = PCNT.int_st.val;
    
    for (int i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            unsigned long current_millis = millis();
            if (got_fg_pulse) {
              fg_period = (current_millis - last_fg_pulse_millis) / PCNT_COUNT_LIMIT;
              if (fg_period > 0) {
                fg_frequency = (1.0/fg_period) * 1000 * 60 / FG_PULSE_PER_REV;
              }
            }
            last_fg_pulse_millis = current_millis;
            got_fg_pulse = true;

            PCNT.int_clr.val = BIT(i);
        }
    }
}

void setup_pulse_counter() {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = PCNT_GPIO,
    .ctrl_gpio_num = -1,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = PCNT_COUNT_LIMIT,
    .counter_l_lim = 0,
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0
  };

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

void set_actuator_pwm(uint32_t val) {
    ctl_pwm_duty = val;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_CTL_MODE, LEDC_CTL_CHANNEL, ctl_pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_CTL_MODE, LEDC_CTL_CHANNEL));
}

void IRAM_ATTR onPIDRegulatorTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  gpio_set_level(PID_TIMING_FLAG_GPIO, 1);

  if (pidreg_enabled) {
    pidreg_config_t pidreg_config_copy = {
      .Kp = pidreg_config.Kp,
      .Ki = pidreg_config.Ki,
      .Kd = pidreg_config.Kd,
      .output_min = pidreg_config.output_min,
      .output_max = pidreg_config.output_max,
      .setpoint = pidreg_config.setpoint
    };

    const double actuator_val = pidreg_compute(
      pidreg_state, &pidreg_config_copy, fg_frequency);

    set_actuator_pwm(round(actuator_val));

    // const uint32_t pid_ki_duty = map(
    //   pidreg_config_copy.Ki,
    //   -1000, 1000,
    //   0, LEDC_TIMER0_DUTY_MAX);
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_CTL_MODE, LEDC_CTL_CHANNEL, ctl_pwm_duty));
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_CTL_MODE, LEDC_CTL_CHANNEL));
  }

  gpio_set_level(PID_TIMING_FLAG_GPIO, 0);

  portEXIT_CRITICAL_ISR(&timerMux);
}

void setPIDRegulatorFrequency(double frequency) {
  int ticks = round(1.0/frequency * 1000000);
  timerAlarmWrite(timer_pid, ticks, true);
}

void setupPIDRegulator() {
  pidreg_state = pidreg_init(ctl_pwm_duty, fg_frequency);

  timer_pid = timerBegin(0, 80, true);
  timerAttachInterrupt(timer_pid, &onPIDRegulatorTimer, true);
  setPIDRegulatorFrequency(timer_freq);
  timerAlarmEnable(timer_pid);
}

static void ledc_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_TIMER0_MODE,
        .duty_resolution  = LEDC_TIMER0_DUTY_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = LEDC_TIMER0_FREQ,
        .clk_cfg          = LEDC_TIMER0_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_ctl_channel = {
        .gpio_num       = LEDC_CTL_GPIO,
        .speed_mode     = LEDC_CTL_MODE,
        .channel        = LEDC_CTL_CHANNEL,
        .intr_type      = LEDC_CTL_INTR,
        .timer_sel      = LEDC_CTL_TIMER,
        .duty           = ctl_pwm_duty,
        .hpoint         = 0,
        .flags = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ctl_channel));  

    ledc_channel_config_t ledc_pid_ki_channel = {
        .gpio_num       = LEDC_PID_KI_GPIO,
        .speed_mode     = LEDC_PID_KI_MODE,
        .channel        = LEDC_PID_KI_CHANNEL,
        .intr_type      = LEDC_PID_KI_INTR,
        .timer_sel      = LEDC_PID_KI_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .flags = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_pid_ki_channel));  
}

void setup() {
  Serial.begin(115200);

  gpio_set_direction(PCNT_GPIO, GPIO_MODE_INPUT);
  gpio_set_direction(PID_TIMING_FLAG_GPIO, GPIO_MODE_OUTPUT);

  ledc_init();

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
            set_actuator_pwm(val);
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
              ctl_pwm_duty,
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