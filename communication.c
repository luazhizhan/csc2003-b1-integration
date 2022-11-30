#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define UART_TX_PIN 0
#define UART_RX_PIN 1

extern Comms_data comms_data;

// Send data via uart to M5StickC
bool repeating_timer_callback(struct repeating_timer *t)
{
    char data[16];
    sprintf(data, "%d,%d,%d,%s\n", comms_data.speed, comms_data.hump, comms_data.dist, comms_data.barcode);
    uart_puts(UART_ID, data);
    return true;
}

// Receive data via uart from M5StickC
void on_uart_rx()
{

    // IMU variables
    uint8_t isImu1 = 0;
    uint8_t isImu2 = 0;
    char imu1[9];
    char imu2[9];
    uint8_t i = 0;

    // Coordinates
    uint8_t isNegative = 0;
    int8_t x = -6;
    int8_t y = -6;

    while (uart_is_readable(UART_ID))
    {
        char ch = uart_getc(UART_ID);

        // IMU 'A' prefix
        // if run to here means detected angle of elevation, speed up
        if ((uint8_t)ch == 65)
        {
            isImu1 = 1;
            // call offset_duty_cycle from PID to speed up
            // set the value to within 30k
            offset_duty_cycle(29000);
            continue;
        }

        // IMU 'B' prefix
        // No angle of elevation, keep moving with default speed
        else if ((uint8_t)ch == 66)
        {
            isImu2 = 1;

            continue;
        }

        // IMU1 data
        if (isImu1)
        {
            imu1[i++] = (uint8_t)ch == 45 ? '-' : ch;
        }

        // IMU2 data
        else if (isImu2)
        {
            imu2[i++] = (uint8_t)ch == 45 ? '-' : ch;
        }

        // Coordinates data
        else
        {

            if ((uint8_t)ch == 45)
            {
                // Set flag
                isNegative = 1;
            }

            else if (isNegative == 1)
            {
                // Set negative number to x or y
                if (x == -6)
                {
                    // convert ascii char to int
                    x = ch - '0';

                    // Negate value
                    x *= -1;
                }
                else
                {
                    // convert ascii char to int
                    y = ch - '0';

                    // Negate value
                    y *= -1;
                }

                // reset flag
                isNegative = 0;
            }
            else
            {
                // Set positivie number to x or y
                if (x == -6)
                {
                    // convert ascii char to int
                    x = ch - '0';
                }
                else
                {
                    // convert ascii char to int
                    y = ch - '0';
                }
            }
        }
    }

    // Send IMU1 data
    if (isImu1)
    {
        imu1[i] = '\0';
        uart_puts(UART_ID, imu1);
    }

    // Send IMU2 data
    else if (isImu2)
    {
        imu2[i] = '\0';
        uart_puts(UART_ID, imu2);
    }

    // Send coordinates data
    else if (x != -6 && y != -6)
    {
        // TODO: Use this
        // gotoNode(int x, int y)

        char text[6];
        sprintf(text, "%d,%d\n", x, y);
        uart_puts(UART_ID, text);
    }
}

int comms_main()
{
    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // Set up repeating timer
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);
}
