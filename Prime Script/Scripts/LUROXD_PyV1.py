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

import uart_custom
from uart_custom import Comm
from uart_custom import init_uart

import sk9822_custom
from sk9822_custom import sk9822_init
from sk9822_custom import set_led_ring

#######################################################################
#                     Global Variable Declaration
#######################################################################

# Speech Control Variables
SpeechLayer = 0
Request = 0
Intent = 0
Objective = 0
Specification = 0
Gesture = 0
DirectIntent = 0

sample_rate   = 16000
record_time   = 2  #s
current_time = 0 # ms

# UART Control Variables
UARTLayer_1_Open = 0x1 # Command Execution
UARTLayer_1_Close = 0x3
UARTLayer_2_Open = 0x2 # Object Detection
UARTLayer_2_Close = 0x4
UARTLayer3_Open = 0x5  # Gestures & Immediate Commands
UARTLayer3_Close = 0x6

# Object Detection Variables
input_size = (224, 224)
labels = ['wallet', 'pliers', 'wrench', 'cup', 'phone', 'screwdriver', 'scissor', 'drill', 'hammer', 'person', 'can', 'bottle']
anchors = [1.31, 3.16, 2.56, 5.34, 5.7, 4.92, 0.94, 1.16, 2.81, 2.25]

class_to_token = {
    'person': 100,
    'phone': 200,
    'wallet': 300,
    'cup': 400,
    'bottle': 500,
    'can': 600,
    'hammer': 700,
    'drill': 800,
    'screwdriver': 900,
    'wrench': 1000,
    'pliers': 1100,
    'scissor': 1200
}

min_token = 100
max_token = 1200
f_max = 15
next_id = 0
tracked_objects = []

def distance(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**0.5

# Semaphores / Flags
SpeechInterrupt = False
ObjectRec = False
SpeechActive = True
ESPConnected = False

#######################################################################
#                         FPIOA Initalization
#######################################################################

fm.register(23,fm.fpioa.I2S0_IN_D0, force=True)
fm.register(19,fm.fpioa.I2S0_WS, force=True)
fm.register(18,fm.fpioa.I2S0_SCLK, force=True)

rx = I2S(I2S.DEVICE_0)
rx.channel_config(rx.CHANNEL_0, rx.RECEIVER, align_mode=I2S.STANDARD_MODE)
rx.set_sample_rate(sample_rate)

sr = isolated_word(dmac=2, i2s=I2S.DEVICE_0, size=15, shift=1) # maix bit set shift=1
sr.set_threshold(150, 150, 9800)

#######################################################################
#                       Function Initalization
#######################################################################
def initalize():
    uart = init_uart()
    comm = Comm(uart)
    sk9822_init()
    Layer0_Load()

    return {"status": "ready"}

#######################################################################
#                           Object Detection
#######################################################################

def main(anchors, labels = None, model_addr="/sd/m.kmodel", sensor_window=input_size, lcd_rotation=0, sensor_hmirror=False, sensor_vflip=False):
    global next_id
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing(sensor_window)
    sensor.set_hmirror(sensor_hmirror)
    sensor.set_vflip(sensor_vflip)
    sensor.run(1)

    try:
        task = None
        task = kpu.load(model_addr)
        kpu.init_yolo2(task, 0.5, 0.3, 5, anchors) # threshold:[0,1], nms_value: [0, 1]
        if ObjectRec == True:
            img = sensor.snapshot()
            t = time.ticks_ms()
            objects = kpu.run_yolo2(task, img)
            t = time.ticks_ms() - t
            new_detections = []
            if objects:
                for obj in objects:
                    rect = obj.rect()
                    x_norm = (rect[0] + rect[2] / 2) / 224
                    y_norm = (rect[1] + rect[3] / 2) / 224
                    a_norm = (rect[2] * rect[3]) / 50176
                    new_detections.append({
                        'rect': rect,
                        'X': x_norm,
                        'Y': y_norm,
                        'A': a_norm,
                        'classid': obj.classid(),
                        'P': obj.value()
                    })

                # Update f for all tracks
                for track in tracked_objects:
                    while track['detection_times'] and current_time - track['detection_times'][0] > 500:
                        track['detection_times'].pop(0)
                    track['f'] = len(track['detection_times'])

                matched_tracks = []
                for det in new_detections:
                    min_dist = float('inf') # Infite Error at first
                    best_track = None

                    # Raw pixel center of detection
                    center_x_det = det['rect'][0] + det['rect'][2] / 2
                    center_y_det = det['rect'][1] + det['rect'][3] / 2

                    # Class token and normalized [0,1]
                    token_det = class_to_token[labels[det['classid']]]
                    norm_c_det = (token_det - min_token) / (max_token - min_token)
                    norm_f_det = 1 / f_max

                    v_det = [
                        det['rect'][0] / 224,
                        det['rect'][1] / 224,
                        det['rect'][2] / 224,
                        det['rect'][3] / 224,

                        norm_c_det,
                        norm_f_det,

                        det['P']
                    ]

                    for track in tracked_objects:
                        if track['missed_frames'] > 0:
                            continue

                        center_x_track = track['rect'][0] + track['rect'][2] / 2
                        center_y_track = track['rect'][1] + track['rect'][3] / 2

                        center_dist = math.sqrt((center_x_det - center_x_track)**2 + (center_y_det - center_y_track)**2)

                        if center_dist < 30:
                            token_track = class_to_token[labels[track['classid']]]
                            norm_c_track = (token_track - min_token) / (max_token - min_token)
                            norm_f_track = track['f'] / f_max

                            v_track = [
                                track['rect'][0] / 224,
                                track['rect'][1] / 224,
                                track['rect'][2] / 224,
                                track['rect'][3] / 224,
                                norm_c_track,
                                norm_f_track,
                                track['P']
                            ]

                            dist = distance(v_det, v_track)
                            if dist < min_dist:
                                min_dist = dist
                                best_track = track

                    if best_track is not None and min_dist < 0.1:
                        # Update existing track
                        best_track['rect'] = det['rect']
                        best_track['X'] = det['X']
                        best_track['Y'] = det['Y']
                        best_track['A'] = det['A']
                        best_track['P'] = det['P']

                        token_diff = abs(token_det - class_to_token[labels[best_track['classid']]])
                        if det['P'] > 0.95 or (token_diff < 200 and det['P'] > best_track['P'] + 0.1):
                            best_track['classid'] = det['classid']

                        best_track['detection_times'].append(current_time)
                        best_track['f'] = len(best_track['detection_times'])
                        best_track['R'] += 1
                        best_track['missed_frames'] = 0
                        matched_tracks.append(best_track)
                else:
                    # New object
                    new_track = {
                        'id': next_id,
                        'rect': det['rect'],
                        'X': det['X'],
                        'Y': det['Y'],
                        'classid': det['classid'],
                        'P': det['P'],
                        'R': 1,
                        'detection_times': [current_time],
                        'f': 1,
                        'missed_frames': 0
                    }
                    tracked_objects.append(new_track)
                    matched_tracks.append(new_track)
                    next_id += 1

                # Update unmatched tracks
                for track in tracked_objects[:]:
                    if track not in matched_tracks:
                        track['missed_frames'] += 1
                        if track['missed_frames'] > 5:
                            tracked_objects.remove(track)

                for track in tracked_objects:
                    if track['missed_frames'] <= 2:
                        UART_Layered_Track(UARTLayer_2_Open, track, UARTLayer_2_Close)

            else:
                # No detections; update tracks
                for track in tracked_objects[:]:
                    track['missed_frames'] += 1
                    if track['missed_frames'] > 5:
                        tracked_objects.remove(track)

    except Exception as e:
        raise e
    finally:
        if not task is None:
            kpu.deinit(task)

#######################################################################
#                           Control Loop
#######################################################################

def control_loop(state):
    while(True):
        current_time = utime.ticks_ms()
        if SpeechLayer == 0: # Wake
            if sr.Done == sr.recognize():
                res = sr.result()
                if res != None:
                    if 0 <= res[0] <= 3: # Wake Word Check
                        SpeechLayer = 1
                        brightness_values = [25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)
                        Layer1_Load() # Loading the next Layer!

        elif SpeechLayer == 1: # Request
            if sr.Done == sr.recognize():
                res = sr.result()
                if res != None:
                    if 0 <= res[0] <= 9: # Request command
                        Request = res[0]
                        SpeechLayer = 2
                        Layer2_Load() # Loading the next layer!
                        brightness_values = [50, 50, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

                    if 10 <= res[0] <= 15: # Direct Command
                        DirectIntent = res[0]
                        UART_Layered_Direct(UARTLayer3_Open, DirectIntent, SpeechLayer, UARTLayer3_Close)
                        SpeechLayer = 0
                        Layer0_Load() # Loading the HOME Layer!
                        brightness_values = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

                    if 16 <= res[0] <= 29: # Gesture Command
                        Gesture = res[0]
                        UART_Layered_Direct(UARTLayer3_Open, Gesture, SpeechLayer, UARTLayer3_Close)
                        SpeechLayer = 0
                        Layer0_Load() # Loading the HOME Layer!
                        brightness_values = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

                    if 30 <= res[0] <= 33:
                        SpeechLayer = 0
                        Layer0_Load() # Loading the HOME Layer!
                        brightness_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

        elif SpeechLayer == 2: # Intent
            if sr.Done == sr.recognize():
                res = sr.result()
                if res != None:
                    if 0 <= res[0] <= 7: # Intent command
                        Intent = res[0]
                        SpeechLayer = 3
                        Layer3_Load() # Loading the next layer!
                        brightness_values = [50, 50, 50, 50, 50, 50, 0, 0, 0, 0, 0, 0]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

                    if 8 <= res[0] <= 21: # Gesture command
                        Gesture = res[0]
                        UART_Layered_Direct(UARTLayer3_Open, Gesture, SpeechLayer, UARTLayer3_Close)
                        SpeechLayer = 0
                        Layer0_Load() # Loading the HOME Layer!
                        brightness_values = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

                    if 22 <= res[0] <= 25: # Interrupt
                        SpeechLayer = 0
                        Layer0_Load() # Loading the HOME Layer!
                        brightness_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

        elif SpeechLayer == 3: # Objective
            if sr.Done == sr.recognize():
                res = sr.result()
                if res != None:
                    if 0 <= res[0] <= 15: # Objective command
                        Objective = res[0]
                        SpeechLayer = 4
                        Layer4_Load()
                        brightness_values = [50, 50, 50, 50, 50, 50, 50, 50, 50, 0, 0, 0]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

                    if 16 <= res[0] <= 19: # Direct command
                        DirectIntent = res[0]
                        UART_Layered_Direct(UARTLayer3_Open, DirectIntent, SpeechLayer, UARTLayer3_Close)
                        SpeechLayer = 0
                        Layer0_Load() # Loading the HOME Layer!
                        brightness_values = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

                    if 20 <= res[0] <= 23: # Interrupt
                        SpeechLayer = 0
                        Layer0_Load() # Loading the HOME Layer!
                        brightness_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)

        elif SpeechLayer == 4: #Specification
            if sr.Done == sr.recognize():
                res = sr.result()
                if res != None:
                    if 0 <= res[0] <= 19: # Specification command
                        Specification = res[0]
                        SpeechLayer = 5
                        ObjectRec = True
                        Layer5_Load()
                        brightness_values = [50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)
                        UART_Layered_Comms(UARTLayer_1_Open, Request, Intent, Objective, Specification, UARTLayer_1_Close)

        elif SpeechLayer == 5: #Interrupt/Object loop
            main(anchors = anchors, labels=labels, model_addr="/sd/model-192544.kmodel")
            if sr.Done == sr.recognize():
                res = sr.result()
                if res != None:
                    if 0 <= res[0] <= 3:
                        SpeechLayer = 0
                        ObjectRec = False
                        Layer0_Load() # Return back to start!
                        brightness_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                        color = [50, 0, 0]  # Red color
                        set_led_ring(brightness_values, color)
                        UART_Layered_Comms(UARTLayer_1_Open, 100, 100, 100, 100, UARTLayer_1_Close) # Force Hand to HALT

if __name__ == "__main__":
    init_state = initalize()
    if init_state.get("status") == "ready":
        control_loop(init_state)
