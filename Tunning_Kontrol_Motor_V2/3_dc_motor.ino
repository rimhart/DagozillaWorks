int motor_position = 0;
int previous_motor_position = 0;
int velocity_start = 0;
int velocity_end = 0;
float time_difference;
float raw_velocity_increment;
float raw_velocity;
float velocity;
int pwm;


void setup_motor() {
  ledcAttachPin(pin_pwm_right, pwm_channel_right);
  ledcAttachPin(pin_pwm_left, pwm_channel_left);
  ledcSetup(pwm_channel_right, pwm_frequency, pwm_resolution);
  ledcSetup(pwm_channel_left, pwm_frequency, pwm_resolution);
  
  digitalWrite(pin_enable_right, HIGH);
  digitalWrite(pin_enable_left, HIGH);  
}

void update_position() {
  int encoder_signal = digitalRead(pin_encoder_B);
  if(encoder_signal > 0){
    motor_position = motor_position + 1;
  }
  else{
    motor_position = motor_position - 1;
  }
}

void compute_raw_velocity() {
  velocity_end = micros();
  time_difference = (velocity_end - velocity_start)/1.0e6;
  raw_velocity_increment = (motor_position - previous_motor_position)/time_difference;
  raw_velocity = raw_velocity_increment/600*60;

  previous_motor_position = motor_position;
  velocity_start = velocity_end;
}

void write_motor(int pwm) {
  if (pwm >= 0) {
    ledcWrite(pwm_channel_left, 0);
    ledcWrite(pwm_channel_right, pwm); 
  }
  else {
    ledcWrite(pwm_channel_right, 0);
    ledcWrite(pwm_channel_left, pwm*-1);
  }
}
