#######################################################################
#
#                  LUROX D Maix-Bit K210 Software
#                  Developed By: Taheemuddin Ahmed
#
#######################################################################

#######################################################################
#                          Speech Functions
#######################################################################
import os
import json
import time
import utime
from Maix import I2S, GPIO
from fpioa_manager import fm
import sensor, image
import array
import math
import struct
from speech_recognizer import isolated_word

sr = isolated_word(dmac=2, i2s=I2S.DEVICE_0, size=35, shift=1) # maix bit set shift=1
sr.set_threshold(100, 15, 11000)

def sr_data_load(index, filename):
    with open('/sd/' + filename, 'rb') as f:
        data = f.read()
    frm_num = len(data) // 26  # Assuming each MFCC coefficient is 2 bytes
    print(frm_num)
    sr.set(index, (frm_num, data))

#######################################################################
#                        Load Memory Functions
#######################################################################

def Layer0_Load():
    # Wake Word
    sr_data_load(0, 'LUROX_0.bin')
    sr_data_load(1, 'LUROX_1.bin')
    sr_data_load(2, 'Azami_0.bin')
    sr_data_load(3, 'Azami_1.bin')

    print("Library Annoucement: Layer 0 Loaded")

def Layer1_Load():
    # Request
    sr_data_load(0, 'Can_0.bin')
    sr_data_load(1, 'Can_1.bin')
    sr_data_load(2, 'Would_0.bin')
    sr_data_load(3, 'Would_1.bin')
    sr_data_load(4, 'Could_0.bin')
    sr_data_load(5, 'Could_1.bin')
    sr_data_load(6, 'Please_0.bin')
    sr_data_load(7, 'Please_1.bin')
    sr_data_load(8, 'May_0.bin')
    sr_data_load(9, 'May_1.bin')

    # Direct
    # sr_data_load(10, 'Grab_0.bin')
    # sr_data_load(11, 'Grab_1.bin')
    # sr_data_load(12, 'Hold_0.bin')
    # sr_data_load(13, 'Hold_1.bin')

    # sr_data_load(14, 'Open_0.bin')
    # sr_data_load(15, 'Open_1.bin')

    # # Gestures
    # sr_data_load(16, 'HighFive_0.bin')
    # sr_data_load(17, 'HighFive_1.bin')
    # sr_data_load(18, 'Peace_0.bin')
    # sr_data_load(19, 'Peace_1.bin')
    # sr_data_load(20, 'Point_0.bin')
    # sr_data_load(21, 'Point_1.bin')
    # sr_data_load(22, 'Rock_0.bin')
    # sr_data_load(23, 'Rock_1.bin')
    # sr_data_load(24, 'ThumbsDown_0.bin')
    # sr_data_load(25, 'ThumbsDown_1.bin')
    # sr_data_load(26, 'ThumbsUp_0.bin')
    # sr_data_load(27, 'ThumbsUp_1.bin')
    # sr_data_load(28, 'Wave_0.bin')
    # sr_data_load(29, 'Wave_1.bin')

    # Interrupt
    sr_data_load(10, 'Stop_0.bin')
    sr_data_load(11, 'Stop_1.bin')
    sr_data_load(12, 'Cancel_0.bin')
    sr_data_load(13, 'Cancel_1.bin')

    print("Library Annoucement: Layer 1 Loaded")


def Layer2_Load():
    # Intention 'Azami CAN you GRAB...'
    sr_data_load(0, 'Grab_0.bin')
    sr_data_load(1, 'Grab_1.bin')
    sr_data_load(2, 'Hold_0.bin')
    sr_data_load(3, 'Hold_1.bin')
    sr_data_load(4, 'Pull_0.bin')
    sr_data_load(5, 'Pull_1.bin')
    sr_data_load(6, 'Push_0.bin')
    sr_data_load(7, 'Push_1.bin')

    # Gestures Ex: 'Azami CAN you WAVE?'
    # sr_data_load(8, 'HighFive_0.bin')
    # sr_data_load(9, 'HighFive_1.bin')
    # sr_data_load(10, 'Peace_0.bin')
    # sr_data_load(11, 'Peace_1.bin')
    # sr_data_load(12, 'Point_0.bin')
    # sr_data_load(13, 'Point_1.bin')
    # sr_data_load(14, 'Rock_0.bin')
    # sr_data_load(15, 'Rock_1.bin')
    # sr_data_load(16, 'ThumbsDown_0.bin')
    # sr_data_load(17, 'ThumbsDown_1.bin')
    # sr_data_load(18, 'ThumbsUp_0.bin')
    # sr_data_load(19, 'ThumbsUp_1.bin')
    # sr_data_load(20, 'Wave_0.bin')
    # sr_data_load(21, 'Wave_1.bin')

    # Interrupt 'Azami CAN you STOP!'
    sr_data_load(8, 'Stop_0.bin')
    sr_data_load(9, 'Stop_1.bin')
    sr_data_load(10, 'Cancel_0.bin')
    sr_data_load(11, 'Cancel_1.bin')

    print("Library Annoucement: Layer 2 Loaded")


def Layer3_Load():
    # Specification
    sr_data_load(0, 'Red_0.bin')
    sr_data_load(1, 'Red_1.bin')
    sr_data_load(2, 'Orange_0.bin')
    sr_data_load(3, 'Orange_1.bin')
    sr_data_load(4, 'Yellow_0.bin')
    sr_data_load(5, 'Yellow_1.bin')
    sr_data_load(6, 'Green_0.bin')
    sr_data_load(7, 'Green_1.bin')
    sr_data_load(8, 'Blue_0.bin')
    sr_data_load(9, 'Blue_1.bin')
    sr_data_load(10, 'Purple_0.bin')
    sr_data_load(11, 'Purple_1.bin')
    sr_data_load(12, 'White_0.bin')
    sr_data_load(13, 'White_1.bin')
    sr_data_load(14, 'Black_0.bin')
    sr_data_load(15, 'Black_1.bin')
    sr_data_load(16, 'Brown_0.bin')
    sr_data_load(17, 'Brown_1.bin')
    sr_data_load(18, 'Grey_0.bin')
    sr_data_load(19, 'Grey_1.bin')

    # Objective (Direct)
    sr_data_load(20, 'Wallet_0.bin')
    sr_data_load(21, 'Wallet_1.bin')
    sr_data_load(22, 'Cup_0.bin')
    sr_data_load(23, 'Cup_1.bin')
    sr_data_load(24, 'Bottle_0.bin')
    sr_data_load(25, 'Bottle_1.bin')
    sr_data_load(26, 'Can_0.bin')
    sr_data_load(27, 'Can_1.bin')
    sr_data_load(28, 'Hammer_0.bin')
    sr_data_load(29, 'Hammer_1.bin')
    sr_data_load(30, 'Wrench_0.bin')
    sr_data_load(31, 'Wrench_1.bin')
    sr_data_load(32, 'Pliers_0.bin')
    sr_data_load(33, 'Pliers_1.bin')
    sr_data_load(34, 'Scissors_0.bin')
    sr_data_load(35, 'Scissors_1.bin')

    # Direct
    sr_data_load(36, 'That_0.bin')
    sr_data_load(37, 'That_1.bin')
    sr_data_load(38, 'This_0.bin')
    sr_data_load(39, 'This_1.bin')

    # Interrupt
    sr_data_load(40, 'Stop_0.bin')
    sr_data_load(41, 'Stop_1.bin')
    sr_data_load(42, 'Cancel_0.bin')
    sr_data_load(43, 'Cancel_1.bin')

    print("Library Annoucement: Layer 3 Loaded")


def Layer4_Load():
    # Object (Specific)
    sr_data_load(0, 'Wallet_0.bin')
    sr_data_load(1, 'Wallet_1.bin')
    sr_data_load(2, 'Cup_0.bin')
    sr_data_load(3, 'Cup_1.bin')
    sr_data_load(4, 'Bottle_0.bin')
    sr_data_load(5, 'Bottle_1.bin')
    sr_data_load(6, 'Can_0.bin')
    sr_data_load(7, 'Can_1.bin')
    sr_data_load(8, 'Hammer_0.bin')
    sr_data_load(9, 'Hammer_1.bin')
    sr_data_load(10, 'Wrench_0.bin')
    sr_data_load(11, 'Wrench_1.bin')
    sr_data_load(12, 'Pliers_0.bin')
    sr_data_load(13, 'Pliers_1.bin')
    sr_data_load(14, 'Scissors_0.bin')
    sr_data_load(15, 'Scissors_1.bin')

    # Interrupt
    sr_data_load(16, 'Stop_0.bin')
    sr_data_load(17, 'Stop_1.bin')
    sr_data_load(18, 'Cancel_0.bin')
    sr_data_load(19, 'Cancel_1.bin')

    print("Library Annoucement: Layer 4 Loaded")


def Interrupt_Load():
    # Interrupt
    sr_data_load(0, 'Stop_0.bin')
    sr_data_load(1, 'Stop_1.bin')
    sr_data_load(2, 'Cancel_0.bin')
    sr_data_load(3, 'Cancel_1.bin')

    print("Library Annoucement: Layer Interrupt Loaded")

# LIBRARY LOADER TESTER
#while True:
    #time.sleep_ms(100)
    #print("Loop Running")
    #Layer1_Load()
    #time.sleep_ms(5)
    #Layer2_Load()
    #time.sleep_ms(5)
    #Layer3_Load()
    #time.sleep_ms(5)
    #Layer4_Load()
    #time.sleep_ms(5)
    #Interrupt_Load()
