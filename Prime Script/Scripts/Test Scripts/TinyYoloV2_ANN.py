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
import sensor, image, lcd
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

#######################################################################
#                     Global Variable Declaration
#######################################################################

import sensor, image, lcd, time, math, gc, sys
import KPU as kpu

# Object Detection Variables
input_size = (224, 224)
labels = ['wallet', 'pliers', 'wrench', 'cup', 'phone', 'screwdriver', 'scissor', 'drill', 'hammer', 'person', 'can', 'bottle']
anchors = [1.31, 3.16, 2.56, 5.34, 5.7, 4.92, 0.94, 1.16, 2.81, 2.25]

class_to_token = {
    'person': 100, 'phone': 200, 'wallet': 300, 'cup': 400, 'bottle': 500, 'can': 600,
    'hammer': 700, 'drill': 800, 'screwdriver': 900, 'wrench': 1000, 'pliers': 1100, 'scissor': 1200
}

min_token = 100
max_token = 1200
f_max = 15.0 # Max repetition frames

# The Engine Room: Your Vector Weights
# Adjust these to change the ANN's behavior.
# Currently set to heavily favor Spatial Position and Historical Class/Repetition.
WEIGHTS = [
    0.6,  # X position (High: must be in the same spot)
    0.6,  # Y position (High: must be in the same spot)
    0.2,  # Width (Low: prevents shrinking box glitches)
    0.2,  # Height (Low: prevents shrinking box glitches)
    0.5,  # Class Token (Medium-High: resists class flickering)
    0.3,  # Probability (Medium)
    0.5   # Repetition (Medium: favors established tracks)
]

def create_vector(rect, classid, prob, rep):
    # Normalize all inputs to 0.0 -> 1.0 space
    x_norm = (rect[0] + rect[2] / 2) / 320.0
    y_norm = (rect[1] + rect[3] / 2) / 240.0
    w_norm = rect[2] / 320.0
    h_norm = rect[3] / 240.0

    token = class_to_token[labels[classid]]
    c_norm = (token - min_token) / (max_token - min_token)

    r_norm = min(rep / f_max, 1.0)

    # Return the raw normalized vector
    return [x_norm, y_norm, w_norm, h_norm, c_norm, prob, r_norm]

def vector_distance(v1, v2):
    # Calculates the Weighted Euclidean Distance
    sum_sq = 0
    for i in range(7):
        diff = (v1[i] - v2[i]) * WEIGHTS[i]
        sum_sq += diff**2
    return math.sqrt(sum_sq)

def vector_strength(v):
    # Calculates the "Winning" magnitude based on Prob and Repetition
    return (v[5] * WEIGHTS[5]) + (v[6] * WEIGHTS[6])

class Track:
    def __init__(self, track_id, det):
        self.id = track_id
        self.rect = det['rect']
        self.classid = det['classid']
        self.prob = det['P']
        self.rep = 1
        self.missed_frames = 0
        self.vector = create_vector(self.rect, self.classid, self.prob, self.rep)

    def update(self, det):
        self.rect = det['rect']
        self.classid = det['classid']
        self.prob = det['P']
        self.rep = min(self.rep + 1, f_max)
        self.missed_frames = 0
        self.vector = create_vector(self.rect, self.classid, self.prob, self.rep)

    def is_trusted(self):
            # RULE: Detected at least 6 times AND prediction score > 60%
            return self.rep >= 6 and self.prob > 0.60

def main(anchors, labels=None, model_addr="/sd/m.kmodel", sensor_window=input_size):
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing(sensor_window)
    sensor.run(1)

    tracked_objects = []
    next_id = 0

    # Tuning Threshold: How "far" apart in the vector space can two objects be
    # before they are considered completely different things?
    DISTANCE_THRESHOLD = 0.35

    try:
        task = kpu.load(model_addr)
        kpu.init_yolo2(task, 0.3, 0.3, 5, anchors)

        while(True):
            img = sensor.snapshot()
            objects = kpu.run_yolo2(task, img)

            new_detections = []
            if objects:
                for obj in objects:
                    new_detections.append({
                        'rect': obj.rect(),
                        'classid': obj.classid(),
                        'P': obj.value(),
                        'vector': create_vector(obj.rect(), obj.classid(), obj.value(), 1)
                    })

            # --- Vector NMS (Conflict Resolution) ---
            # If two raw detections are too close in the vector space, kill the weaker one.
            # This solves the "Person vs Bottle" flickering in a single frame.
            filtered_detections = []
            while new_detections:
                current_det = new_detections.pop(0)
                keep = True
                for other_det in new_detections:
                    dist = vector_distance(current_det['vector'], other_det['vector'])
                    if dist < DISTANCE_THRESHOLD:
                        # They are occupying the same logical space. Who wins?
                        str_current = vector_strength(current_det['vector'])
                        str_other = vector_strength(other_det['vector'])
                        if str_other > str_current:
                            keep = False
                            break # The other detection beat this one
                if keep:
                    filtered_detections.append(current_det)

            # --- Temporal Tracking ---
            matched_tracks = []
            for det in filtered_detections:
                best_track = None
                min_dist = DISTANCE_THRESHOLD

                for track in tracked_objects:
                    if track.missed_frames > 2:
                        continue

                    dist = vector_distance(det['vector'], track.vector)
                    if dist < min_dist:
                        min_dist = dist
                        best_track = track

                if best_track is not None:
                    # Update existing track vector
                    best_track.update(det)
                    matched_tracks.append(best_track)

                else:
                    # Spawn new track vector
                    new_track = Track(next_id, det)
                    tracked_objects.append(new_track)
                    matched_tracks.append(new_track)
                    next_id += 1

            # Prune and Draw
            COASTING_LIMIT = 15

            for track in tracked_objects[:]:
                if track not in matched_tracks:
                    track.missed_frames += 1
                    # Decay the repetition vector if we miss a frame
                    track.rep = max(track.rep - 1, 0)

                    survival_limit = COASTING_LIMIT if track.is_trusted() else 2

                    if track.missed_frames > 5:
                        tracked_objects.remove(track)

            for track in tracked_objects:
                if track.missed_frames == 0:
                    img.draw_rectangle(track.rect, color=(0, 255, 0))
                    img.draw_string(track.rect[0], track.rect[1], "%s: %.2f R:%d" %(labels[track.classid], track.prob, track.rep), scale=1.5, color=(255, 0, 0))



    except Exception as e:
        sys.print_exception(e)
    finally:
        if not task is None:
            kpu.deinit(task)

if __name__ == "__main__":
    try:
        main(anchors = anchors, labels=labels, model_addr=0x300000)
    except Exception as e:
        sys.print_exception(e)
    finally:
        gc.collect()
