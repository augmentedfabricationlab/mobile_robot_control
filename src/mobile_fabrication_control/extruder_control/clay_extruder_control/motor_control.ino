/*
  Arduino motor control functions for extruder control 
  created on 22.02.2021
*/

void motor_init() {
  motor.setEnablePin(enable_pin);
  motor.setPinsInverted(false, false, true);
  pinMode(m1_pin, OUTPUT);
  pinMode(m2_pin, OUTPUT);
  pinMode(m3_pin, OUTPUT);
  digitalWrite(m1_pin, HIGH);
  digitalWrite(m2_pin, HIGH);
  digitalWrite(m3_pin, LOW);
  motor.setMaxSpeed(motordata.motor_maxspeed);
  motor.setAcceleration(40000);
  motor.setSpeed(motordata.motor_speed); 
  Serial.println("Motor setup executed");
}

void motor_setup() {
  motor.setMaxSpeed(motordata.motor_maxspeed);
  motor.setSpeed(motordata.motor_speed); 
  Serial.println("Motor setup executed");
}

void check_motor() {
  if ((motordata.motor_state == 1) and (OUTPUT_STATE == 1)) { 
    motor.runSpeed();
    if (motor_state_print.hasPassed(10000)) {
      motor_state_print.restart();
      Serial.println("Motor is running...");
    }
  }
  else if ((motordata.motor_state == 1) and (OUTPUT_STATE == 0)) {
    motor.enableOutputs();
    OUTPUT_STATE = 1;
  }
  else if ((motordata.motor_state == 0) and (OUTPUT_STATE == 1)) {
    motor.disableOutputs();
    OUTPUT_STATE = 0;
  }
  else {
    motor.stop();
  }
}
