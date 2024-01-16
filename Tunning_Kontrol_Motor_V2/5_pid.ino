float kp = 0;
float ki = 0;
float kd = 0;

float error = 0;
float previous_error = 0;
float error_total = 0;
float error_change = 0;


void update_pid_parameters(){
  kp = Serial.parseInt();
  ki = Serial.parseInt();
  kd = Serial.parseInt();

  start_time = micros();
  timer_1_start = 0;

  error_total = 0;

  while (Serial.available()) Serial.read();
}

int pid_control(int set_point, float motor_velocity) {
  error = set_point - motor_velocity;
  error_total = error_total + error*time_difference;
  error_change = (error - previous_error)/time_difference;

  int motor_input = kp*error + ki*error_total + kd*error_change;

  previous_error = error;

  return constrain(motor_input, -255, 255);
}
