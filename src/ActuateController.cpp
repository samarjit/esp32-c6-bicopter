/**
 * Input (0, 0) → no PWM → centered

Input (-90, -90) → full backward current

Input (90, 90) → full forward current


 */

#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "ActuateController.h"

// Actuator 1 H-Bridge pins
#define ACT1_IN1  0  // PWM capable
#define ACT1_IN2  1  // PWM capable

// Actuator 2 H-Bridge pins
#define ACT2_IN1  3  // PWM capable
#define ACT2_IN2  4  // PWM capable

#define PWM_FREQ     1000   // 20 kHz for quiet operation
#define PWM_RES      8      // 10-bit resolution (0-1023)
#define MCPWM_FREQ     50   // 5 kHz for motors
#define MCPWM_RES      8      // 8-bit resolution (0-255)

// Handles for MCPWM operator and comparators
// static mcpwm_cmpr_handle_t act1_in1_cmpr = NULL;
// static mcpwm_cmpr_handle_t act1_in2_cmpr = NULL;
// static mcpwm_cmpr_handle_t act2_in1_cmpr = NULL;
// static mcpwm_cmpr_handle_t act2_in2_cmpr = NULL;
// static mcpwm_oper_handle_t oper0 = NULL;
// static mcpwm_oper_handle_t oper1 = NULL;


// int getJoystickInput1();
// int getJoystickInput2();
/*
void setupActuators() {

  
  pinMode(ACT1_IN1, OUTPUT);
  pinMode(ACT1_IN2, OUTPUT);
  pinMode(ACT2_IN1, OUTPUT);
  pinMode(ACT2_IN2, OUTPUT);
  if (ledcAttach(ACT1_IN1, PWM_FREQ, PWM_RES) == 0) {
    Serial.println("Failed to attach ACT1_IN1");
  } 
  if (ledcAttach(ACT1_IN2, PWM_FREQ, PWM_RES) == 0) {
    Serial.println("Failed to attach ACT1_IN2");
  }
  if (ledcAttach(ACT2_IN1, PWM_FREQ, PWM_RES) == 0) {
    Serial.println("Failed to attach ACT2_IN1");
  }
  if (ledcAttach(ACT2_IN2, PWM_FREQ, PWM_RES) == 0) {
    Serial.println("Failed to attach ACT2_IN2");
  }

}

void setActuatorPWM(uint8_t actuator, int inputAngle) {
  inputAngle = constrain(inputAngle, -90, 90);
  int pwmVal = map(abs(inputAngle), 0, 30, 0, 255);  // Map to 10-bit duty

  if (actuator == 0) {
    if (inputAngle > 0) {
      ledcWrite(ACT1_IN1, pwmVal);  // Forward
      ledcWrite(ACT1_IN2, 0);
      // analogWrite(ACT1_IN1, pwmVal);
      // analogWrite(ACT1_IN2, LOW);
    } else if (inputAngle < 0) {
      ledcWrite(ACT1_IN1, 0);
      ledcWrite(ACT1_IN2, pwmVal);  // Backward
      // analogWrite(ACT1_IN1, LOW);
      // analogWrite(ACT1_IN2, pwmVal);
    } else {
      ledcWrite(ACT1_IN1, 0);
      ledcWrite(ACT1_IN2, 0);
      // analogWrite(ACT1_IN1, LOW);
      // analogWrite(ACT1_IN2, LOW);
    }
  }

  if (actuator == 1) {
    if (inputAngle > 0) {
      ledcWrite(ACT2_IN1, pwmVal);  // Forward
      ledcWrite(ACT2_IN2, 0);
      // analogWrite(ACT2_IN1, pwmVal);
      // analogWrite(ACT2_IN2, LOW);
    } else if (inputAngle < 0) {
      ledcWrite(ACT2_IN1, 0);
      ledcWrite(ACT2_IN2, pwmVal);  // Backward
      // analogWrite(ACT2_IN1, LOW);
      // analogWrite(ACT2_IN2, pwmVal);
    } else {
      ledcWrite(ACT2_IN1, 0);
      ledcWrite(ACT2_IN2, 0);
      // analogWrite(ACT2_IN1, LOW);
      // analogWrite(ACT2_IN2, LOW);
    }
  }
}

*/

void setupActuators() {
    // Initialize MCPWM units and timers
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ACT1_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, ACT1_IN2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, ACT2_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, ACT2_IN2);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = MCPWM_FREQ;
    pwm_config.cmpr_a = 0;    // initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    // initial duty cycle of PWMxB = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

void setActuatorPWM(uint8_t actuator, int inputAngle) {
    inputAngle = constrain(inputAngle, -90, 90);
    float duty = map(abs(inputAngle), 0, 30, 0, 100);  // Duty cycle in percent

    if (actuator == 0) {
        if (inputAngle > 0) {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
        } else if (inputAngle < 0) {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
        } else {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
        }
    }

    if (actuator == 1) {
        if (inputAngle > 0) {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
        } else if (inputAngle < 0) {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty);
        } else {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
        }
    }
}
    

/*   

void setupActuators() {
    // MCPWM timer config
    mcpwm_timer_handle_t timer0 = NULL;
    mcpwm_timer_handle_t timer1 = NULL;

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1 MHz, so period is in microseconds
        .period_ticks = 1000000 / MCPWM_FREQ, // period in ticks for desired freq
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_config, &timer0);
    mcpwm_new_timer(&timer_config, &timer1);

    // MCPWM operator config
    mcpwm_operator_config_t oper_config = { .group_id = 0 };
    mcpwm_new_operator(&oper_config, &oper0);
    mcpwm_new_operator(&oper_config, &oper1);

    // Connect operator to timer
    mcpwm_operator_connect_timer(oper0, timer0);
    mcpwm_operator_connect_timer(oper1, timer1);

    // Create comparators for each output
    mcpwm_comparator_config_t cmpr_config = {};
    mcpwm_new_comparator(oper0, &cmpr_config, &act1_in1_cmpr);
    mcpwm_new_comparator(oper0, &cmpr_config, &act1_in2_cmpr);
    mcpwm_new_comparator(oper1, &cmpr_config, &act2_in1_cmpr);
    mcpwm_new_comparator(oper1, &cmpr_config, &act2_in2_cmpr);

    // Create generators for each output
    mcpwm_gen_handle_t act1_in1_gen, act1_in2_gen, act2_in1_gen, act2_in2_gen;
    mcpwm_generator_config_t gen_config = {};

    gen_config.gen_gpio_num = ACT1_IN1;
    mcpwm_new_generator(oper0, &gen_config, &act1_in1_gen);
    gen_config.gen_gpio_num = ACT1_IN2;
    mcpwm_new_generator(oper0, &gen_config, &act1_in2_gen);
    gen_config.gen_gpio_num = ACT2_IN1;
    mcpwm_new_generator(oper1, &gen_config, &act2_in1_gen);
    gen_config.gen_gpio_num = ACT2_IN2;
    mcpwm_new_generator(oper1, &gen_config, &act2_in2_gen);

    // Set generator actions: high at timer start, low at comparator match
    mcpwm_generator_set_action_on_timer_event(act1_in1_gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(act1_in1_gen, act1_in1_cmpr, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_COMPARE_EVENT_EQUAL, MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_timer_event(act1_in2_gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(act1_in2_gen, act1_in2_cmpr, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_COMPARE_EVENT_EQUAL, MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_timer_event(act2_in1_gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(act2_in1_gen, act2_in1_cmpr, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_COMPARE_EVENT_EQUAL, MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_timer_event(act2_in2_gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(act2_in2_gen, act2_in2_cmpr, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_COMPARE_EVENT_EQUAL, MCPWM_GEN_ACTION_LOW));

    // Start timers
    mcpwm_timer_enable(timer0);
    mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP);
    mcpwm_timer_enable(timer1);
    mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP);
}

void setActuatorPWM(uint8_t actuator, int inputAngle) {
    inputAngle = constrain(inputAngle, -90, 90);
    // Map inputAngle to duty in microseconds (period_ticks)
    // For 5kHz, period_ticks = 200, so duty = 0..200
    int period_ticks = 1000000 / MCPWM_FREQ;
    int duty_ticks = map(abs(inputAngle), 0, 90, 0, period_ticks);

    if (actuator == 0) {
        if (inputAngle > 0) {
            mcpwm_comparator_set_compare_value(act1_in1_cmpr, duty_ticks);
            mcpwm_comparator_set_compare_value(act1_in2_cmpr, 0);
        } else if (inputAngle < 0) {
            mcpwm_comparator_set_compare_value(act1_in1_cmpr, 0);
            mcpwm_comparator_set_compare_value(act1_in2_cmpr, duty_ticks);
        } else {
            mcpwm_comparator_set_compare_value(act1_in1_cmpr, 0);
            mcpwm_comparator_set_compare_value(act1_in2_cmpr, 0);
        }
    }

    if (actuator == 1) {
        if (inputAngle > 0) {
            mcpwm_comparator_set_compare_value(act2_in1_cmpr, duty_ticks);
            mcpwm_comparator_set_compare_value(act2_in2_cmpr, 0);
        } else if (inputAngle < 0) {
            mcpwm_comparator_set_compare_value(act2_in1_cmpr, 0);
            mcpwm_comparator_set_compare_value(act2_in2_cmpr, duty_ticks);
        } else {
            mcpwm_comparator_set_compare_value(act2_in1_cmpr, 0);
            mcpwm_comparator_set_compare_value(act2_in2_cmpr, 0);
        }
    }
}

*/