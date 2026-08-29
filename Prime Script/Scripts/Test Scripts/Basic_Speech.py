#######################################################################
#
#                  LUROX D Maix-Bit K210 Software
#                  Developed By: Taheemuddin Ahmed
#
#######################################################################

#######################################################################
#                    Libraries & Modules Import
#######################################################################

import os
import json
import machine
import time
import utime
from Maix import I2S, GPIO
from fpioa_manager import fm
import sensor, image
import array
import math
import struct
from machine import UART
import KPU as kpu
import gc, sys
from speech_recognizer import isolated_word
print("Packages now loaded")

#######################################################################
#                     SD Card Initalization & Load
#######################################################################

os.chdir('/sd')
dirs = os.listdir()
sd_files = os.listdir('/sd')

sys.path.append('/sd')
print("sys.path now includes:", sys.path)
#sys.path.append('/sd/lib')

import speechrec_custom
from speechrec_custom import Layer0_Load
from speechrec_custom import Layer1_Load
from speechrec_custom import Layer2_Load
from speechrec_custom import Layer3_Load
from speechrec_custom import Layer4_Load
from speechrec_custom import Interrupt_Load
from speechrec_custom import sr

import uart_custom
from uart_custom import Comm
from uart_custom import init_uart

import sk9822_custom
from sk9822_custom import sk9822_init
from sk9822_custom import set_led_ring
print("Custom packages are now loaded")

#######################################################################
#                     Global Variable Declaration
#######################################################################

sample_rate   = 16000
record_time   = 1  #s
current_time = 0 # ms

print("Global Variables are now loaded")

#######################################################################
#                         FPIOA Initalization
#######################################################################

fm.register(23,fm.fpioa.I2S0_IN_D0, force=True)
fm.register(19,fm.fpioa.I2S0_WS, force=True)
fm.register(18,fm.fpioa.I2S0_SCLK, force=True)

rx = I2S(I2S.DEVICE_0)
rx.channel_config(rx.CHANNEL_0, rx.RECEIVER, align_mode=I2S.STANDARD_MODE)
rx.set_sample_rate(sample_rate)

#sr = isolated_word(dmac=2, i2s=I2S.DEVICE_0, size=15, shift=1) # maix bit set shift=1
sr.set_threshold(10, 10, 10000)
sr.run()

print("Speech has been configured")

#######################################################################
#                           Control Loop
#######################################################################

def initalize():
    sk9822_init()
    Layer0_Load()
    return {"status": "ready"}

gc.collect()

def control_loop(state):

    SpeechLayer = 0

    while(True):
        if SpeechLayer == 0: # Wake
            if sr.Done == sr.recognize():
                res = sr.result()
                print(sr.result())
                if res != None:
                    if 0 <= res[0] <= 3: # Wake Word Check
                        sr.reset()
                        time.sleep_ms(20)
                        SpeechLayer = 1
                        brightness_values = [250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)
                        Layer1_Load() # Loading the next Layer!
                        gc.collect()
                        print("Awoken")

        elif SpeechLayer == 1:
            if sr.Done == sr.recognize():
                res = sr.result()
                print(sr.result())
                if res != None:
                    if 0 <= res[0] <= 9: # Wake Word Check
                        sr.reset()
                        SpeechLayer = 2
                        brightness_values = [25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25]
                        color = [50, 0, 50]
                        set_led_ring(brightness_values, color)
                        Layer2_Load() # Loading the next Layer!
                        print("Request")

                    elif 10 <= res[0] <= 15:
                        sr.reset()
                        SpeechLayer = 2
                        brightness_values = [25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25]
                        color = [50, 0, 50]
                        set_led_ring(brightness_values, color)
                        Layer0_Load() # Loading the next Layer!
                        print("Direct")

                    elif 16 <= res[0] <= 29:
                        sr.reset()
                        SpeechLayer = 2
                        brightness_values = [25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25]
                        color = [50, 0, 50]
                        set_led_ring(brightness_values, color)
                        Layer0_Load() # Loading the next Layer!
                        print("Gestures")

                    elif 30 <= res[0] <= 33:
                        sr.reset()
                        SpeechLayer = 2
                        brightness_values = [25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25]
                        color = [50, 0, 50]
                        set_led_ring(brightness_values, color)
                        Layer0_Load() # Loading the next Layer!
                        print("STOP")


initalize()
control_loop(1)
