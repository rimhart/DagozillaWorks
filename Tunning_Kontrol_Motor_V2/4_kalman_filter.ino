#define R 100     // measurement constant
#define Q 1       // process variance constant

float Xt, Xt_update, Xt_prev;
float Pt, Pt_update;
float Pt_prev = 1;
float Kt;

float kalman_filter(float Data) {
  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  
  Kt = Pt_update / (Pt_update + R);
  Xt = Xt_update + ( Kt * (Data - Xt_update));
  Pt = (1 - Kt) * Pt_update;
  
  Xt_prev = Xt;
  Pt_prev = Pt;

  return Xt;       // KalmanFilterData
}
