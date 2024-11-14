#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "freertos/task.h"
//#include "freertos/queue.h"
#include "driver/gpio.h"

//GPIO pin definitions:
#define rear_fwd GPIO_NUM_4
#define rear_rev GPIO_NUM_16
#define front_fwd GPIO_NUM_17
#define front_rev GPIO_NUM_18
#define rear_clock GPIO_NUM_19
#define rear_data GPIO_NUM_21
#define front_data GPIO_NUM_22
#define front_clock GPIO_NUM_23
#define right_dn GPIO_NUM_25
#define right_up GPIO_NUM_33
#define left_up GPIO_NUM_35
#define left_dn GPIO_NUM_39

//Brifter states
enum State {
    UP,
    DOWN,
    BASE,
    CANCEL
}

//Groupset specific information
const int r_speeds = 6;
const int r_positions[] = {500, 525, 550, 575, 600, 625};
int r_gear = 0;
const int f_speeds = 2;
const int f_positions[] = {500, 550};
int f_gear = 0;
int new_data = 0;

struct Caliper {
    const int cycleTime;
    unsigned volatile clockFlag;
    long lastInterrupt;
    long value;
    long now;
    long finalValue;
    int sign;
    int currentBit;
    unsigned char dataIn;
    gpio_num_t dataPin;
}

struct Motor {
  gpio_num_t fwdPin;
  gpio_num_t revPin;
  long last_t;
  int pidIntegral;
}

//Interupt handler for position reading
static void IRAM_ATTR caliper_isr(struct Caliper c) {
  c.now = esp_timer_get_time();
  c.clockFlag = 1;
  new_data = 1;
  c.dataIn = gpio_get_level(c.data);
  vTaskDelete(NULL);
}

void decode(struct Caliper c) {
    if((c.now - c.lastInterrupt) > c.cycleTime) {
      long new_v = c.value * c.sign;
      // pid_calc(c.motor, new_v, c.finalValue);
        c.finalValue = new_v
        c.currentBit = 0;
        c.value = 0;
        c.sign = 1;
    } else if (c.currentBit < 16 ) {
        if (c.dataIn == 0) {
            if (c.currentBit < 16) {
                c.value |= 1 << c.currentBit;
            } else if (c.currentBit == 20) {
                c.sign = -1;
            }       
        }
    c.currentBit++;
   }
   c.lastInterrupt = c.now;
}

void init_caliper(struct Caliper c, gpio_num_t data){
    c.cycleTime = 32000;
    c.clockFlag = 0;
    c.lastInterrupt = 0;
    c.value = 0;
    c.finalValue = 0;
    c.sign = 1;
    c.currentBit = 1;
    c.dataPin = data;
}

void gpio_setup(struct Caliper front, struct Caliper rear){
    gpio_set_direction(left_up, GPIO_MODE_INPUT);
    gpio_set_direction(left_dn, GPIO_MODE_INPUT);
    gpio_set_direction(right_up, GPIO_MODE_INPUT);
    gpio_set_direction(right_dn, GPIO_MODE_INPUT);
    gpio_set_direction(front_fwd, GPIO_MODE_OUTPUT);
    gpio_set_direction(front_rev, GPIO_MODE_OUTPUT);
    
    gpio_install_isr_service(0);

    gpio_set_intr_type(front_clock, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(rear_clock, GPIO_INTR_POSEDGE);

    gpio_isr_handler_add(front_clock, caliper_isr, front);
    gpio_isr_handler_add(rear_clock, caliper_isr, rear);

    gpio_intr_enable(front_clock);
    gpio_intr_enable(rear_clock);
}

void PID_calc(struct Motor m, long pos, long pre_pos) {
  /* int error;
    int Kp = 1, Ki = 1, Kd = 1;
    error = setpoint - *m.position_ptr;
    if(abs(error) > 5){
      int d_t = esp_timer_get_time() - m.last_t;
      m.pidIntegral += (d_t) * error;
      int derivative = () / d_t
      }*/
}

void app_main(void) {
    struct Caliper front_caliper;
    struct Caliper rear_caliper;

    char left_buttons[] = {0, 0};
    char right_buttons[] = {0, 0};
    int rear_speed = 0;
    int front_speed = 0;
    enum State left_state = BASE;
    enum State right_state = BASE;

    init_caliper(front_caliper, front_data);
    init_caliper(rear_caliper, rear_data);

    gpio_setup(front_caliper, rear_caliper);
    
    while(0 == 0) {
      vTaskDelay(configTICK_RATE_HZ / 1000);
      if(new_data){
	printf("%d\t%d\n", front_caliper.now, rear_caliper.now);
	new_data = 0;
      }
      /*if(left_buttons[UP] == 0 && left_buttons[DOWN] == 0) {
            if(left_state == UP && f_gear < (f_speeds - 1)) f_gear++;
            if(left_state == DOWN && f_gear != 0) f_gear--;
            left_state = BASE;
        }
        else if(left_buttons[UP] == 1 && left_buttons[DOWN] == 0 && left_state == BASE) left_state = UP;
        else if(left_buttons[UP] == 0 && left_buttons[DOWN] == 1 && left_state == BASE) left_state = DOWN;
        else if(left_buttons[UP] == 1 && left_buttons[DOWN] == 1) left_state = CANCEL;
        
        if(right_buttons[UP] == 0 && right_buttons[DOWN] == 0) {
            if(right_state == UP && r_gear < (r_speeds - 1)) r_gear++;
            if(right_state == DOWN && r_gear != 0) r_gear--;
            right_state = BASE;
        }
        else if(right_buttons[UP] == 1 && right_buttons[DOWN] == 0 && right_state == BASE) right_state = UP;
        else if(right_buttons[UP] == 0 && right_buttons[DOWN] == 1 && right_state == BASE) right_state = DOWN;
        else if(right_buttons[UP] == 1 && right_buttons[DOWN] == 1) right_state = CANCEL;

        if(front_caliper.clockFlag) {
            decode(front_caliper);
            front_caliper.clockFlag = 0;
        }
        if(rear_caliper.clockFlag) {
            decode(rear_caliper);
            rear_caliper.clockFlag = 0;
	    }*/
    }
}
