#if !defined(__FG_TACHOMETER_H__)
#define __FG_TACHOMETER_H__

#include <Arduino.h>

#include <math.h>
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

typedef struct {
  int unit;
  uint32_t status;
  unsigned long timeStamp;
} pcnt_evt_t;

int calculate_rpm(int firstTime, int lastTime, int pulseTotal, int pulsePerRev);

class FGTachometer
{
public:
    FGTachometer(int, int, int);
    int getRPM(int);
    void init();
private:
    int pulse_gpio_num;
    int lastStamp0 = 0;
    int pulseCnt;
    int pulsePerRev;
    QueueHandle_t pcnt_evt_queue;
};

#endif // __FG_TACHOMETER_H__
