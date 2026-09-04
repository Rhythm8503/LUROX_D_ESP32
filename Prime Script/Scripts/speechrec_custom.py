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

def Layer0_Load(): # Wake Word
    sr_data_load(0, 'LUROX_0.bin')
    sr_data_load(1, 'LUROX_1.bin')
    sr_data_load(2, 'Azami_0.bin')
    sr_data_load(3, 'Azami_1.bin')

    print("Library Annoucement: Layer 0 Loaded")

def Layer1_Load(): # Request
    sr_data_load(0, 'Can_0.bin')            #0
    sr_data_load(1, 'Can_1.bin')            #0
    sr_data_load(2, 'Would_0.bin')          #1
    sr_data_load(3, 'Would_1.bin')          #1
    sr_data_load(4, 'Could_0.bin')          #2
    sr_data_load(5, 'Could_1.bin')          #2
    sr_data_load(6, 'Please_0.bin')         #3
    sr_data_load(7, 'Please_1.bin')         #3
    
    # Direct
    sr_data_load(8, 'Grab_0.bin')           #4
    sr_data_load(9, 'Grab_1.bin')           #4
    sr_data_load(10, 'Hold_0.bin')          #5
    sr_data_load(11, 'Hold_1.bin')          #5

    # # Gestures
    sr_data_load(12, 'Wave_0.bin')          #6
    sr_data_load(13, 'Wave_1.bin')          #6
    sr_data_load(14, 'Hand_Shake_0.bin')    #7
    sr_data_load(15, 'Hand_Shake_1.bin')    #7
    sr_data_load(16, 'ThumbsUp_0.bin')      #8
    sr_data_load(17, 'ThumbsUp_1.bin')      #8
    sr_data_load(18, 'ThumbsDown_0.bin')    #9
    sr_data_load(19, 'ThumbsDown_1.bin')    #9
    sr_data_load(20, 'HighFive_0.bin')      #10
    sr_data_load(21, 'HighFive_1.bin')      #10

    # Interrupt
    sr_data_load(22, 'Stop_0.bin')          #11
    sr_data_load(23, 'Stop_1.bin')          #11
    sr_data_load(24, 'Cancel_0.bin')        #12
    sr_data_load(25, 'Cancel_1.bin')        #12

    print("Library Annoucement: Layer 1 Loaded")

def Layer2_Load(): # Intention
    sr_data_load(0, 'Grab_0.bin')           #0
    sr_data_load(1, 'Grab_1.bin')           #0
    sr_data_load(2, 'Hold_0.bin')           #1
    sr_data_load(3, 'Hold_1.bin')           #1
    sr_data_load(4, 'Pull_0.bin')           #2
    sr_data_load(5, 'Pull_1.bin')           #2
    sr_data_load(6, 'Push_0.bin')           #3
    sr_data_load(7, 'Push_1.bin')           #3

    # Gestures Ex: 'Azami CAN you WAVE?'
    sr_data_load(8, 'Wave_0.bin')           #4
    sr_data_load(9, 'Wave_1.bin')           #4
    sr_data_load(10, 'Hand_Shake_0.bin')    #5
    sr_data_load(11, 'Hand_Shake_1.bin')    #5
    sr_data_load(12, 'ThumbsUp_0.bin')      #6
    sr_data_load(13, 'ThumbsUp_1.bin')      #6
    sr_data_load(14, 'ThumbsDown_0.bin')    #7
    sr_data_load(15, 'ThumbsDown_1.bin')    #7
    sr_data_load(16, 'HighFive_0.bin')      #8
    sr_data_load(17, 'HighFive_1.bin')      #8
    sr_data_load(18, 'Point_0.bin')         #9
    sr_data_load(19, 'Point_1.bin')         #9
    sr_data_load(20, 'Peace_0.bin')         #10
    sr_data_load(21, 'Peace_1.bin')         #10    
    sr_data_load(22, 'Ok!_0.bin')           #11
    sr_data_load(23, 'Ok!_1.bin')           #12


    # Interrupt 'Azami CAN you STOP!'
    sr_data_load(24, 'Stop_0.bin')           #13
    sr_data_load(25, 'Stop_1.bin')           #13
    sr_data_load(26, 'Cancel_0.bin')        #14
    sr_data_load(27, 'Cancel_1.bin')        #14

    print("Library Annoucement: Layer 2 Loaded")

def Layer3_Load(): #Specification
    sr_data_load(0, 'Red_0.bin')            #0
    sr_data_load(1, 'Red_1.bin')            #0
    sr_data_load(2, 'Orange_0.bin')         #1
    sr_data_load(3, 'Orange_1.bin')         #1
    sr_data_load(4, 'Yellow_0.bin')         #2
    sr_data_load(5, 'Yellow_1.bin')         #2
    sr_data_load(6, 'Green_0.bin')          #3
    sr_data_load(7, 'Green_1.bin')          #3
    sr_data_load(8, 'Blue_0.bin')           #4
    sr_data_load(9, 'Blue_1.bin')           #4
    sr_data_load(10, 'Purple_0.bin')        #5
    sr_data_load(11, 'Purple_1.bin')        #5
    sr_data_load(12, 'White_0.bin')         #6
    sr_data_load(13, 'White_1.bin')         #6
    sr_data_load(14, 'Black_0.bin')         #7
    sr_data_load(15, 'Black_1.bin')         #7
    sr_data_load(16, 'Brown_0.bin')         #8
    sr_data_load(17, 'Brown_1.bin')         #8
    sr_data_load(18, 'Grey_0.bin')          #9
    sr_data_load(19, 'Grey_1.bin')          #9  

    # Objective (Direct)
    sr_data_load(20, 'Wallet_0.bin')        #10
    sr_data_load(21, 'Wallet_1.bin')        #10
    sr_data_load(22, 'Pliers_0.bin')        #11
    sr_data_load(23, 'Pliers_1.bin')        #11
    sr_data_load(24, 'Wrench_0.bin')        #12
    sr_data_load(25, 'Wrench_1.bin')        #12
    sr_data_load(26, 'Cup_0.bin')           #13
    sr_data_load(27, 'Cup_1.bin')           #13
    sr_data_load(28, 'Phone_0.bin')         #14
    sr_data_load(29, 'Phone_1.bin')         #14
    sr_data_load(30, 'Screwdrivers_0.bin')  #15
    sr_data_load(31, 'Screwdrivers_1.bin')  #15
    sr_data_load(34, 'Scissors_0.bin')      #16
    sr_data_load(35, 'Scissors_1.bin')      #16
    sr_data_load(36, 'Drill_0.bin')         #17
    sr_data_load(37, 'Drill_1.bin')         #17
    sr_data_load(38, 'Hammer_0.bin')        #18
    sr_data_load(39, 'Hammer_1.bin')        #18
    sr_data_load(40, 'Can_0.bin')           #19
    sr_data_load(41, 'Can_1.bin')           #19
    sr_data_load(42, 'Bottle_0.bin')        #20
    sr_data_load(43, 'Bottle_1.bin')        #20

    # Interrupt
    sr_data_load(44, 'Stop_0.bin')          #21
    sr_data_load(45, 'Stop_1.bin')          #21
    sr_data_load(46, 'Cancel_0.bin')        #22
    sr_data_load(47, 'Cancel_1.bin')        #22

    print("Library Annoucement: Layer 3 Loaded")

def Layer4_Load(): # Objective 
    sr_data_load(0, 'Wallet_0.bin')         #0
    sr_data_load(1, 'Wallet_1.bin')         #0
    sr_data_load(2, 'Pliers_0.bin')         #1
    sr_data_load(3, 'Pliers_1.bin')         #1
    sr_data_load(4, 'Wrench_0.bin')         #2
    sr_data_load(5, 'Wrench_1.bin')         #2
    sr_data_load(6, 'Cup_0.bin')            #3
    sr_data_load(7, 'Cup_1.bin')            #3
    sr_data_load(8, 'Phone_0.bin')          #4
    sr_data_load(9, 'Phone_1.bin')          #4
    sr_data_load(10, 'Screwdrivers_0.bin')  #5
    sr_data_load(11, 'Screwdrivers_1.bin')  #5
    sr_data_load(12, 'Scissors_0.bin')      #6
    sr_data_load(13, 'Scissors_1.bin')      #6
    sr_data_load(14, 'Drill_0.bin')         #7
    sr_data_load(15, 'Drill_1.bin')         #7
    sr_data_load(16, 'Hammer_0.bin')        #8
    sr_data_load(17, 'Hammer_1.bin')        #8
    sr_data_load(18, 'Can_0.bin')           #9
    sr_data_load(19, 'Can_1.bin')           #9
    sr_data_load(20, 'Bottle_0.bin')        #10
    sr_data_load(21, 'Bottle_1.bin')        #10

    # Interrupt
    sr_data_load(22, 'Stop_0.bin')          #11
    sr_data_load(23, 'Stop_1.bin')          #11
    sr_data_load(24, 'Cancel_0.bin')        #12
    sr_data_load(25, 'Cancel_1.bin')        #12 

    print("Library Annoucement: Layer 4 Loaded")

def Interrupt_Load(): # Interrupt
    
    sr_data_load(0, 'Stop_0.bin')       #0
    sr_data_load(1, 'Stop_1.bin')       #0
    sr_data_load(2, 'Cancel_0.bin')     #1
    sr_data_load(3, 'Cancel_1.bin')     #1

    print("Library Annoucement: Layer Interrupt Loaded")

