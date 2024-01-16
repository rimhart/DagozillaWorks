int set_point = 0;
float amplitude = 80;

/*
int set_point_function(int input, int periode) {
  float s = sin(2*PI*input/periode);
  return (int)(s*amplitude);
}
*/

int set_point_function(int input, int periode) {
  return amplitude;
}


void change_set_point(){
  if (current_time <= set_point_period){
    set_point = set_point_function(current_time, set_point_period);
  }
  else {
    set_point = 0;
  }
}
