// set point is a single sine function with period 2 s
#define set_point_period 4000000              // 4 second
#define change_set_point_time 10000          // 10 milisecond

int start_time;
int current_time;
int timer_1_start;
bool timer_1;


void check_timer_1() {
  if (current_time - timer_1_start >= change_set_point_time) {
    timer_1 = true;
    timer_1_start = current_time;
  }
  else {
    timer_1 = false;
  }
}
