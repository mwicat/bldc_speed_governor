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


FGTachometer* tachometer;

const int pulse_gpio_num = 19;

const int pulseCnt = 10;
const int pulsePerRev = 4;


void setup() {
  Serial.begin(115200);

  tachometer = new FGTachometer(pulse_gpio_num, pulseCnt, pulsePerRev);
  tachometer->init();

}


void loop() {
    int rpm = tachometer->getRPM(1000);
    printf("RPM:%d\n", rpm);
}



extern "C" void app_main() {
    setup();
    for(;;) {
        loop();
    }
}