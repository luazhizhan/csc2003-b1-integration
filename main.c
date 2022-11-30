#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include "pid.c"
#include "communication.h"
#include "ultrasonic.h"
#include "mapping.c"

#define PI 3.14159

const int UART_TX = 0;
const int UART_RX = 1;
const int WHEEL_ENCODER_RIGHT = 5;
const int WHEEL_ENCODER_LEFT = 6;
const int ULTRA_FRONT_TRIG = 9;
const int ULTRA_FRONT_ECHO = 10;
const int ULTRA_LEFT_TRIG = 11;
const int ULTRA_LEFT_ECHO = 12;
const int ULTRA_RIGHT_TRIG = 14;
const int ULTRA_RIGHT_ECHO = 15;
const int PWM_LEFT = 16;
const int PWM_RIGHT = 17;
const int INPUT_A_LEFT = 18;
const int INPUT_B_LEFT = 19;
const int INPUT_A_RIGHT = 20;
const int INPUT_B_RIGHT = 21;
const int BARCODE_IR = 28;

const int DEGREE_PER_NOTCH = 9;

extern Comms_data comms_data;

int main()
{
    comms_main();
    comms_data.speed = 0;
    comms_data.hump = 0;
    comms_data.dist = 0;
    comms_data.barcode[0] = '0';
    pid_main();
    Map();

    while (1)
    {

        tight_loop_contents();
    }
}
