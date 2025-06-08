
#include "ServoController.h"
#include <Arduino.h>
#include "driver/ledc.h"

#define ACTUATOR1_PIN  8  // PWM pin for actuator Left
#define ACTUATOR2_PIN  9  // PWM pin for actuator Right

// LEDC Configuration
#define LEDC_CHANNEL1  0
#define LEDC_CHANNEL2  1
#define LEDC_TIMER     0
#define PWM_FREQ       50      // 50 Hz for servo
#define PWM_RES        16      // 16-bit resolution

// Servo pulse width range for -90 to +90 degrees
#define SERVO_MIN_US   1000
#define SERVO_MAX_US   2000
#define SERVO_CENTER_US 1500

// Convert microseconds to duty cycle for 16-bit resolution (1 << PWM_RES) - 1) is 65535
#define US_TO_DUTY(us) ((us) * ((1 << PWM_RES) - 1) / (1000000 / PWM_FREQ))


int getJoystickInput1();
int getJoystickInput2();

void setupMyServoConroller() {
  // Initialize LEDC for both actuators
  ledcAttach(ACTUATOR1_PIN, PWM_FREQ, PWM_RES);
  
  ledcAttach(ACTUATOR2_PIN, PWM_FREQ, PWM_RES);
  

  // Center both actuators initially
  setMyServoAngle(0, 0);
}

void loop1() {
  // Replace with your actual input method
  int input1 = getJoystickInput1();  // e.g., range -90 to +90
  int input2 = getJoystickInput2();  // same

  setMyServoAngle(input1, input2);

  delay(20);  // Small delay for servo stability
}

// smooth self centering logic

// int currentAngle1 = 0, currentAngle2 = 0;

// void loop2() {
//   int target1 = getJoystickInput1();
//   int target2 = getJoystickInput2();

//   // Smooth transition (simple linear interpolation)
//   currentAngle1 += (target1 - currentAngle1) / 5;
//   currentAngle2 += (target2 - currentAngle2) / 5;

//   setActuatorAngle(currentAngle1, currentAngle2);
//   delay(20);
// }



// Maps angle to pulse width and writes PWM
void setMyServoAngle(int angle1, int angle2) {
  angle1 = constrain(angle1, -90, 90);
  angle2 = constrain(angle2, -90, 90);

  int us1 = map(angle1, -90, 90, SERVO_MIN_US, SERVO_MAX_US);
  int us2 = map(angle2, -90, 90, SERVO_MIN_US, SERVO_MAX_US);

  uint32_t duty1 = US_TO_DUTY(us1);
  uint32_t duty2 = US_TO_DUTY(us2);

  ledcWrite(ACTUATOR1_PIN, duty1);
  ledcWrite(ACTUATOR2_PIN, duty2);  
}

// Dummy input for testing
int getJoystickInput1() {
  return 0;  // Replace with actual analog input or app input
}

int getJoystickInput2() {
  return 0;  // Replace with actual analog input or app input
}
