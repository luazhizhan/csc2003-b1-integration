#include "pico/stdlib.h"

const int INPUT_A_LEFT = 18;
const int INPUT_B_LEFT = 19;
const int INPUT_A_RIGHT = 20;
const int INPUT_B_RIGHT = 21;
const int WHEEL_ENCODER_RIGHT = 5;
const int WHEEL_ENCODER_LEFT = 6;
const int PWM_LEFT = 17;
const int PWM_RIGHT = 16;
const int DEGREE_PER_NOTCH = 9;

bool is_turning = false;

float wheel_left_rotation = 0.0;
float wheel_right_rotation = 0.0;
float global_rpm = 0.0;

int time_elapsed = 0;

u_short PWM_RIGHT_CYCLE = 32768;
u_short PWM_LEFT_CYCLE = 32768;

int degrees_to_turn = 0;
int counter_notches_turn = 0;

struct repeating_timer timer;

PID pid_control_left, pid_control_right, *pid_ctrl_ptr_left, *pid_ctrl_ptr_right;

void PID_Coefficents(struct PID *pid_struct, float Kp, float Ki, float Kd, float tau, float intergrator_lmt)
{
    pid_struct->Kp = Kp;
    pid_struct->Ki = Ki;
    pid_struct->Kd = Kd;
    pid_struct->tau = tau;
    pid_struct->integrator_lmt = intergrator_lmt;
    pid_struct->time_prev = get_absolute_time();
}

void change_pwm(float difference, int direction)
{
    int pwm_duty_cycle_per_rpm = 180; // Calibrated value
    // Resets PWM cycle to make it 50% duty if it goes above 65535 otherwise we will reset to 0
    if (PWM_RIGHT_CYCLE > 65000 || PWM_LEFT_CYCLE > 65000)
    {
        PWM_RIGHT_CYCLE = 32678;
        PWM_RIGHT_CYCLE = 32678;
    }
    PWM_RIGHT_CYCLE += round(difference * pwm_duty_cycle_per_rpm);
}

void stop_movement()
{
    // Stops all movement (directional or otherwise)
    gpio_put(INPUT_A_LEFT, 0);
    gpio_put(INPUT_A_RIGHT, 0);
    gpio_put(INPUT_B_LEFT, 0);
    gpio_put(INPUT_B_RIGHT, 0);
}

void turn_left(int degrees)
{
    // Alternate channel to turn
    gpio_put(INPUT_A_LEFT, 1);
    gpio_put(INPUT_A_RIGHT, 1);
    gpio_put(INPUT_B_LEFT, 0);
    gpio_put(INPUT_B_RIGHT, 0);
    degrees_to_turn = degrees;
    is_turning = true;
}

void turn_right(int degrees)
{
    // Alternate channel to turn
    gpio_put(INPUT_A_LEFT, 0);
    gpio_put(INPUT_A_RIGHT, 0);
    gpio_put(INPUT_B_LEFT, 1);
    gpio_put(INPUT_B_RIGHT, 1);
    degrees_to_turn = degrees;
    is_turning = true;
}

void move_forward()
{
    // Set channel to high to move motor
    gpio_put(INPUT_A_LEFT, 1);
    gpio_put(INPUT_A_RIGHT, 0);
    gpio_put(INPUT_B_LEFT, 0);
    gpio_put(INPUT_B_RIGHT, 1);
}

void calculate_pwm_change_pwm()
{
    if (!is_turning)
    {
        pid_ctrl_ptr_right = &pid_control_right;
        pid_ctrl_ptr_right->setpoint = wheel_left_rotation;
        pid_ctrl_ptr_right->measured = wheel_right_rotation;
        absolute_time_t curr_time = get_absolute_time();
        float diff_right = PID_Compute(curr_time, pid_ctrl_ptr_right);
        change_pwm(diff_right, 1);
    }
}

float distance_travelled(int time_elapsed)
{
    int distance_per_interval = 2 * PI * 3;                                                                      // r = 3, calibrated, interval of 1 second
    return ((wheel_left_rotation / distance_per_interval) + (wheel_right_rotation / distance_per_interval)) / 2; // returns average distance in cm
}

bool timer_callback(struct repeating_timer *t)
{
    calculate_pwm_change_pwm();
    time_elapsed += 1;
    return 0;
}

void offset_duty_cycle(int duty_cycle_offset)
{ // Takes in a int
    if (PWM_RIGHT_CYCLE + duty_cycle_offset > 65535 || PWM_LEFT_CYCLE + duty_cycle_offset > 65535)
    {
        PWM_RIGHT_CYCLE += duty_cycle_offset;
        PWM_LEFT_CYCLE += duty_cycle_offset;
    }
    else
    { // Sets PWM to default value so we don't overflow
        PWM_RIGHT_CYCLE = 32768;
        PWM_LEFT_CYCLE = 32768;
        PWM_RIGHT_CYCLE += duty_cycle_offset;
        PWM_LEFT_CYCLE += duty_cycle_offset;
    }
}

void set_duty_cycle()
{
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_RIGHT));
    pwm_set_gpio_level(PWM_RIGHT, PWM_RIGHT_CYCLE);
    pwm_set_gpio_level(PWM_LEFT, PWM_LEFT_CYCLE);
}

void wheel_speed_right()
{
    if (is_turning)
    {
        if (counter_notches_turn * DEGREE_PER_NOTCH >= degrees_to_turn)
        {
            stop_movement();
            degrees_to_turn = 0;
            counter_notches_turn = 0;
            is_turning = false;
        }
        else
        {
            counter_notches_turn += 1;
        }
    }
    else
    {
        wheel_right_rotation += 0.05;
    }
}

void wheel_speed_left()
{
    // only need to check on one wheel since both are turning
    if (is_turning)
    {
        if (counter_notches_turn * DEGREE_PER_NOTCH >= degrees_to_turn)
        {
            stop_movement();
            degrees_to_turn = 0;
            counter_notches_turn = 0;
            is_turning = false;
        }
        else
        {
            counter_notches_turn += 1;
        }
    }
    else
    {
        wheel_left_rotation += 0.05;
    }
}

void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == WHEEL_ENCODER_RIGHT)
    {
        wheel_speed_right();
    }
    if (gpio == WHEEL_ENCODER_LEFT)
    {
        wheel_speed_left();
    }
}

float PID_Compute(absolute_time_t time, struct PID *pid)
{
    float time_delta = absolute_time_diff_us(pid->time_prev, time) / 1000000.f; // seconds
    float error = pid->setpoint - pid->measured;

    // Proportional
    float proportional = pid->Kp * error;

    // Intergral
    //  (previous + current*0.5)
    float intergral = pid->integrator + (0.5f * pid->Ki * pid->prev_err * time_delta);

    // Clamp it to prevent wind-up
    if (intergral > +(pid->integrator_lmt))
        intergral = +pid->integrator_lmt;
    else if (intergral < -(pid->integrator_lmt))
        intergral = -pid->integrator_lmt;

    pid->integrator = intergral;

    // Differential
    //  Band limited differentiator using the tau term
    //  Differential of the measured term, prevents BIG jumps on setpoint change
    float err_diff = -(2.0f * pid->Kd * (pid->measured - pid->prev_measured) + (2.0f * pid->tau - time_delta) * pid->differentiator) / (2.0f * pid->tau + time_delta);

    pid->differentiator = err_diff;

    // Store what we need for next time
    pid->prev_err = error;
    pid->time_prev = time;
    pid->prev_measured = pid->measured;

    // Output is PID equation: Kp*error + Ki*error_integrated + Kd*error_differential
    float output = proportional + pid->integrator + pid->differentiator;
    return (output);
}

int pid_main()
{
    stdio_init_all();
    // Init PID struct
    PID_Coefficents(&pid_control_right, 2.5, 0.1, 0.05, 0.02, 1);

    // Init GPIO required
    gpio_init(WHEEL_ENCODER_RIGHT);
    gpio_init(WHEEL_ENCODER_LEFT);
    gpio_init(INPUT_A_LEFT);
    gpio_init(INPUT_B_LEFT);
    gpio_init(INPUT_A_RIGHT);
    gpio_init(INPUT_B_RIGHT);

    // Set PWM pins
    gpio_set_function(PWM_RIGHT, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LEFT, GPIO_FUNC_PWM);

    // Set GPIO dir to in for wheel encoder
    // Set GPIO to out power board
    gpio_set_dir(WHEEL_ENCODER_RIGHT, GPIO_IN);
    gpio_set_dir(WHEEL_ENCODER_LEFT, GPIO_IN);
    gpio_set_dir(INPUT_A_LEFT, GPIO_OUT);
    gpio_set_dir(INPUT_B_LEFT, GPIO_OUT);
    gpio_set_dir(INPUT_A_RIGHT, GPIO_OUT);
    gpio_set_dir(INPUT_B_RIGHT, GPIO_OUT);

    // Set GPIO for power board to low
    gpio_put(INPUT_A_LEFT, 0);
    gpio_put(INPUT_B_LEFT, 0);
    gpio_put(INPUT_A_RIGHT, 0);
    gpio_put(INPUT_B_RIGHT, 0);

    // Pull up on wheel encoders
    gpio_pull_up(WHEEL_ENCODER_RIGHT);
    gpio_pull_up(WHEEL_ENCODER_LEFT);

    // Find out which PWM slice is connected to GPIO 14
    uint slice_num = pwm_gpio_to_slice_num(PWM_RIGHT);

    // Enabled Interrupts on wheel encoders
    gpio_set_irq_enabled_with_callback(WHEEL_ENCODER_RIGHT, GPIO_IRQ_EDGE_FALL, 1, &gpio_callback);
    gpio_set_irq_enabled(WHEEL_ENCODER_LEFT, GPIO_IRQ_EDGE_FALL, 1);

    // Setup PWM and interrupt
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, set_duty_cycle);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get PWM config
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    pwm_init(slice_num, &config, true);

    // Sets timer to fire every 1s
    add_repeating_timer_ms(1000, timer_callback, NULL, &timer);

    return 0;
}
