#include "pidreg.h"


pidreg_state_t* pidreg_init(double actuator_val, double sensor_val) {
    pidreg_state_t* pidreg_state = (pidreg_state_t*) malloc(sizeof(pidreg_state_t));

    pidreg_state->integral_sum = actuator_val;
    pidreg_state->last_sensor_val = sensor_val;

    return pidreg_state;
}

double pidreg_compute(pidreg_state_t* pidreg_state, pidreg_config_t* pidreg_config, double sensor) {
    double sensor_diff = sensor - pidreg_state->last_sensor_val;
    double error = pidreg_config->setpoint - sensor;

    double p_term = pidreg_config->Kp * error;

    pidreg_state->integral_sum += pidreg_config->Ki * error;
    pidreg_state->integral_sum = PID_CONSTRAIN(
        pidreg_state->integral_sum,
        pidreg_config->output_min,
        pidreg_config->output_max);

    double i_term = pidreg_state->integral_sum;

    double d_term = -pidreg_config->Kd * sensor_diff;
    
    double actuator = p_term + i_term + d_term;

    actuator = PID_CONSTRAIN(
        actuator,
        pidreg_config->output_min,
        pidreg_config->output_max);

    pidreg_state->last_sensor_val = sensor;

    return actuator;
}
