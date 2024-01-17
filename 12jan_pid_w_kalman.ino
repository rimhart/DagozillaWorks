// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

// Pins

#define ENCA 2 // YELLOW / D00
#define ENCB 3 // WHITE
#define PWM_R 6
#define PWM_L 7
#define ENA_R 8
#define ENA_L 9

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
float eprev = 0;

// Compute the control signal u
//DEFINE PID
float kp = 9.79516608785605;
float ki = 62.5249393058061;
float kd = 0.377645572278455;

// pid sudut kecil
float kp_30 = 10;
float ki_30 = 0.01;
float kd_30 = 0;
float u;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

// kalman filter
float KalmanFilterData;
float Xt, Xt_update, Xt_prev, Pt, Pt_update, Pt_prev, Kt, R, Q;

void setup() {
  Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(ENA_R,OUTPUT);
  pinMode(ENA_L,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);

  R = 100;
  Q = 1;
  Pt_prev = 1;
}

void loop() {

  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;

  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/600.0*60.0;
  float v2 = velocity2/600.0*60.0;

  //Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  // float vt = 100;
  float vt = 100*(sin(prevT/1e6));

    // kalman
  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  Kt = Pt_update / (Pt_update + R);
  Xt = Xt_update + (Kt * (v1Prev - Xt_update));
  Pt = (1 - Kt) * Pt_update;
  Xt_prev = Xt;
  Pt_prev = Pt;
  KalmanFilterData = Xt;

  // float e = vt-v1Filt;
  float e = vt-KalmanFilterData;
  eintegral = eintegral + e*deltaT;
  float dedt = (e-eprev)/(deltaT);
  
  // if (e < 40) {
  //   u = kp_30*e + ki_30*eintegral + kd_30*dedt;
  // }
  // else {
  u = kp*e + ki*eintegral + kd*dedt;
  // }
  

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }

  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }


  setMotor(dir,pwr);
  eprev = e;

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(KalmanFilterData);
  Serial.print(" ");
  Serial.print(e);
  Serial.print(" ");
  // Serial.print(pwr);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal){
  
  if(dir == 1){
    analogWrite(PWM_R,pwmVal);
    analogWrite(PWM_L, 0);
    digitalWrite(ENA_R,HIGH);
    digitalWrite(ENA_L,HIGH);
  }
  else if(dir == -1){
    analogWrite(PWM_L,pwmVal);
    analogWrite(PWM_R, 0);
    digitalWrite(ENA_R,HIGH);
    digitalWrite(ENA_L,HIGH);
  }
  else{
    digitalWrite(ENA_R,LOW);
    digitalWrite(ENA_L,LOW);
  }  
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}