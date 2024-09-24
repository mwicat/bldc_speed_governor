#if !defined(__PIDREG_H__)
#define __PIDREG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

#define PID_CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double output_min;
    double output_max;
    double setpoint;
} pidreg_config_t;

typedef struct {
    double integral_sum;
    double last_sensor_val;
} pidreg_state_t;

pidreg_state_t* pidreg_init(double actuator_val, double sensor_val);
double pidreg_compute(pidreg_state_t* pidreg_state, pidreg_config_t* pidreg_config, double input);

#ifdef __cplusplus
}
#endif


#endif // __PIDREG_H__
