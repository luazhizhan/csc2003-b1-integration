#define GPIO_PIN_FRONT_TRIG 10
#define GPIO_PIN_FRONT_ECHO 11
#define GPIO_PIN_LEFT_TRIG 12
#define GPIO_PIN_LEFT_ECHO 13
#define GPIO_PIN_RIGHT_TRIG 14
#define GPIO_PIN_RIGHT_ECHO 15

int timeout = 26100;

void setupUltrasonicPins(void)
{
    gpio_init(GPIO_PIN_FRONT_ECHO);
    gpio_init(GPIO_PIN_FRONT_TRIG);
    gpio_init(GPIO_PIN_LEFT_ECHO);
    gpio_init(GPIO_PIN_LEFT_TRIG);
    gpio_init(GPIO_PIN_RIGHT_ECHO);
    gpio_init(GPIO_PIN_RIGHT_TRIG);
    gpio_set_dir(GPIO_PIN_FRONT_ECHO, GPIO_IN);
    gpio_set_dir(GPIO_PIN_FRONT_TRIG, GPIO_OUT);
    gpio_set_dir(GPIO_PIN_LEFT_ECHO, GPIO_IN);
    gpio_set_dir(GPIO_PIN_LEFT_TRIG, GPIO_OUT);
    gpio_set_dir(GPIO_PIN_RIGHT_ECHO, GPIO_IN);
    gpio_set_dir(GPIO_PIN_RIGHT_TRIG, GPIO_OUT);
}

uint64_t getPulse(uint trigPin, uint echoPin)
{
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint64_t width = 0;

    while (gpio_get(echoPin) == 0)
        tight_loop_contents();
    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1)
    {
        width++;
        sleep_us(1);
        if (width > timeout)
            return 0;
    }
    absolute_time_t endTime = get_absolute_time();

    return absolute_time_diff_us(startTime, endTime);
}

uint64_t scanForward()
{
    uint64_t forwardDistance = getPulse(GPIO_PIN_FRONT_TRIG, GPIO_PIN_FRONT_ECHO);
    return forwardDistance / 29 / 2;
}

uint64_t scanLeft()
{
    uint64_t leftDistance = getPulse(GPIO_PIN_LEFT_TRIG, GPIO_PIN_LEFT_ECHO);
    return leftDistance / 29 / 2;
}

uint64_t scanRight()
{
    uint64_t rightDistance = getPulse(GPIO_PIN_RIGHT_TRIG, GPIO_PIN_RIGHT_ECHO);
    return rightDistance / 29 / 2;
}
