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

#include "FGTachometer.h"

#include <PID_v1.h>


FGTachometer* tachometer;
PID* pid_regulator;

const int pulse_gpio_num = 19;

const int pulseCnt = 5;
const int pulsePerRev = 4;

double Kp = 0.1;
double Ki = 0.1;
double Kd = 0;

const int POn = 0;

double rpm_setpoint = 200;
double rpm_measured;

const int pwm_channel = 0;
const int pwm_gpio_num = 13;
const int pwm_resolution = 8;
const int pwm_frequency = 7000;
double pwm_duty = 127;


void setup() {
  gpio_set_drive_capability((gpio_num_t)pwm_gpio_num, GPIO_DRIVE_CAP_3);

  Serial.begin(115200);

  tachometer = new FGTachometer(pulse_gpio_num, pulseCnt, pulsePerRev);
  tachometer->init();

  pid_regulator = new PID(
    &rpm_measured, &pwm_duty, &rpm_setpoint,
    Kp, Ki, Kd, POn, DIRECT);
  pid_regulator->SetMode(AUTOMATIC);
  pid_regulator->SetOutputLimits(1, 255);

  ledcSetup(pwm_channel, pwm_frequency, pwm_resolution);
  ledcAttachPin(pwm_gpio_num, pwm_channel);
  ledcWrite(pwm_channel, pwm_duty);
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

void loop() {
  char line_buf[255];

  bool got_input = readLineSerial(line_buf, sizeof(line_buf));
  if (got_input) {
    double Kp_scan, Ki_scan, Kd_scan;

    if (sscanf(line_buf, "%lf,%lf,%lf", &Kp_scan, &Ki_scan, &Kd_scan) != 3) {
      Serial.printf("Error parsing input '%s'\n", line_buf);
    } else {
      Kp = Kp_scan;
      Ki = Ki_scan;
      Kd = Kd_scan;
      //Serial.printf("New tuning: Kp=%.2lf Ki=%.2lf Kd=%.2lf\n", Kp, Ki, Kd);
      pid_regulator->SetTunings(Kp, Ki, Kd);
    }
  }
    rpm_measured = tachometer->getRPM(1000);

    pid_regulator->Compute();

    Serial.printf("%lu,%.2f,%.2f\n", millis(), rpm_measured, pwm_duty);
    ledcWrite(pwm_channel, pwm_duty); 
}

extern "C" void app_main() {
    setup();
    for(;;) {
        loop();
    }
}