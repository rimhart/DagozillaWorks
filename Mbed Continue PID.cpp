#include "mbed.h"

// Pins
DigitalIn ENCA(D2);
DigitalIn ENCB(D3);
PwmOut PWM_R(D6);
PwmOut PWM_L(D7);
DigitalOut ENA_R(D8);
DigitalOut ENA_L(D9);

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
float eprev = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

// kalman filter
float KalmanFilterData;
float Xt, Xt_update, Xt_prev, Pt, Pt_update, Pt_prev, Kt, R, Q;

void readEncoder();

int main() {

    ENCA.mode(PullUp);
    ENCB.mode(PullUp);

    PWM_R.period(0.001); // Set PWM period to 1 ms (equivalent to 1 kHz)
    PWM_L.period(0.001);

    R = 100;
    Q = 1;
    Pt_prev = 1;

    ENCA.rise(&readEncoder);

    while (1) {
        // read the position and velocity
        int pos = 0;
        float velocity2 = 0;
        __disable_irq(); // disable interrupts temporarily while reading
        pos = pos_i;
        velocity2 = velocity_i;
        __enable_irq(); // turn interrupts back on

        // Compute velocity with method 1
        long currT = us_ticker_read();
        float deltaT = ((float)(currT - prevT)) / 1.0e6;
        float velocity1 = (pos - posPrev) / deltaT;
        posPrev = pos;
        prevT = currT;

        // Convert count/s to RPM
        float v1 = velocity1 / 600.0 * 60.0;
        float v2 = velocity2 / 600.0 * 60.0;

        // Low-pass filter (25 Hz cutoff)
        v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
        v1Prev = v1;
        v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
        v2Prev = v2;

        // kalman
        Xt_update = Xt_prev;
        Pt_update = Pt_prev + Q;
        Kt = Pt_update / (Pt_update + R);
        Xt = Xt_update + (Kt * (v1Filt - Xt_update));
        Pt = (1 - Kt) * Pt_update;
        Xt_prev = Xt;
        Pt_prev = Pt;
        KalmanFilterData = Xt;

        // float vt = 50+50*(sin(prevT/1e6)>0); //ini setpoint kotak
        //buat trapesium

        // Compute the control signal u
        //DEFINE PID

        //INI PID BUAT SETPOINT LURUS
        //float vt = 100; //ini setpoint lurus
        float vt = 100;
        float kp = 4.7679;
        float ki = 87.3169;
        float kd = 0.0035374;

        //INI PID BUAT SETPOINT SINUSOIDAL
        // float vt = 50*sin(prevT/1e6);
        // float kp = 20;
        // float ki = 1;
        // float kd = 0.775;

        // float e = vt-v1Filt;
        float e = vt - KalmanFilterData;
        eintegral = eintegral + e * deltaT;
        float dedt = (e - eprev) / (deltaT);

        float u = kp * e + ki * eintegral + kd * dedt;

        // Set the motor speed and direction
        int dir = 1;
        if (u < 0) {
            dir = -1;
        }
        int pwr = (int)fabs(u);
        if (pwr > 1.0) {
            pwr = 1.0;
        }

        setMotor(dir, pwr);
        eprev = e;

        printf("%f %f\n", vt, KalmanFilterData);
        wait_ms(1);
    }
}

void setMotor(int dir, float pwmVal) {
    if (dir == 1) {
        PWM_R = pwmVal;
        PWM_L = 0.0;
        ENA_R = 1;
        ENA_L = 1;
    } else if (dir == -1) {
        PWM_L = pwmVal;
        PWM_R = 0.0;
        ENA_R = 1;
        ENA_L = 1;
    } else {
        ENA_R = 0;
        ENA_L = 0;
    }
}

void readEncoder() {
    // Read encoder B when ENCA rises
    int b = ENCB.read();
    int increment = 0;
    if (b > 0) {
        // If B is high, increment forward
        increment = 1;
    } else {
        // Otherwise, increment backward
        increment = -1;
    }
    pos_i = pos_i + increment;

    // Compute velocity with method 2
    long currT = us_ticker_read();
    float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
    velocity_i = increment / deltaT;
    prevT_i = currT;
}
