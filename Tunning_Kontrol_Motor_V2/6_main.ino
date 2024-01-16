void setup() {
  Serial.begin(BAUD);
  Serial.println();
  Serial.println(PROG_NAME);

  pinMode(pin_enable_right, OUTPUT);
  pinMode(pin_enable_left, OUTPUT);
  pinMode(pin_pwm_right, OUTPUT);
  pinMode(pin_pwm_left, OUTPUT);
  pinMode(pin_encoder_A, INPUT);
  pinMode(pin_encoder_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(pin_encoder_A), update_position, RISING);

  setup_motor();

  start_time = micros();
  timer_1_start = 0;
}


void loop() {
  current_time = micros() - start_time;
  check_timer_1();

  if (Serial.available()) {
    update_pid_parameters();
  }

  if (timer_1 == true) {
    change_set_point();
  }

  compute_raw_velocity();
  velocity = kalman_filter(raw_velocity);
  pwm = pid_control(set_point, velocity);
  write_motor(pwm);
  print_data();
  delay(1);
}


void print_data(){
  Serial.print(set_point);
  Serial.print(" ");
  Serial.print(velocity);
//  Serial.print(" ");
//  Serial.print(error);
//  Serial.print(" ");
//  Serial.print(pwm);
  Serial.println();
}
