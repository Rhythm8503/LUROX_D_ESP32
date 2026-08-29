#######################################################################
#
#                  LUROX D Maix-Bit K210 Software
#                  Developed By: Taheemuddin Ahmed
#
#######################################################################

import machine
import time
import utime
from Maix import I2S, GPIO
from fpioa_manager import fm

# Define GPIO pins for data and clock
DAT_PIN = 24
CLK_PIN = 25

# Register pins
fm.register(DAT_PIN, fm.fpioa.GPIOHS24)
fm.register(CLK_PIN, fm.fpioa.GPIOHS25)

# Initialize GPIO pins
dat_pin = GPIO(GPIO.GPIOHS25, GPIO.OUT)
clk_pin = GPIO(GPIO.GPIOHS24, GPIO.OUT)

# The Pins are flipped on the end as it seems to work, likely a fault of the PCB design.

# Define the number of LEDs in the ring
NUM_LEDS = 12

def sk9822_init():
    """
    Initialize the SK9822 LED ring by setting the GPIO pins to output mode.
    """
    dat_pin.value(0)
    clk_pin.value(0)

def sk9822_send_data(data):
    """
    Send 32-bit data to the SK9822 LED ring.

    :param data: 32-bit data value to send.
    """
    for i in range(31, -1, -1):
        clk_pin.value(0)
        time.sleep_us(1)  # Short delay to mimic nop in C
        dat_pin.value((data >> i) & 1)
        clk_pin.value(1)
        time.sleep_us(1)  # Short delay to mimic nop in C

def sk9822_start_frame():
    """
    Start the SK9822 frame by sending a 32-bit zero.
    """
    sk9822_send_data(0x00000000)

def sk9822_stop_frame():
    """
    Stop the SK9822 frame by sending a 32-bit one.
    """
    sk9822_send_data(0xFFFFFFFF)

def set_led_ring(brightness_values, color):
    """
    Control the LED ring with specified brightness values and color.

    :param brightness_values: List of 12 integers representing the brightness for each LED (0-255).
    :param color: List of 3 integers representing the RGB color values (0-255).
    """
    if len(brightness_values) != NUM_LEDS or len(color) != 3:
        raise ValueError("Invalid input length for brightness or color.")

    set_color = (color[2] << 16) | (color[1] << 8) | color[0]

    led_data = []
    for brightness in brightness_values:
        if brightness > 1:
            led_value = (((0xe0 | (brightness * 2)) << 24) | set_color)
        else:
            led_value = 0xe0000000
        led_data.append(led_value)

    fm.unregister(18) # Disable Clock
    send_led_data(led_data) # Activate LED Array
    fm.register(18,fm.fpioa.I2S0_SCLK, force=True) # Enable Clock

def send_led_data(led_data):
    """
    Send the prepared LED data to the LED ring using the SK9822 protocol.

    :param led_data: List of LED data values to send.
    """
    sk9822_start_frame()
    for data in led_data:
        sk9822_send_data(data)
    sk9822_stop_frame()