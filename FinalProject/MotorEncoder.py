import RPi.GPIO as GPIO
import sys
import math
import os

# Use BCM pin numbering
GPIO.setmode(GPIO.BCM)
ENC_SPEED_PATH = "/sys/module/EncoderDriver/parameters/enc_speed"

def read_encoder():
    f = open(ENC_SPEED_PATH, "r")
    enc_value = abs(int(f.readline()))
    f.close()

    return enc_value

# Main loop
try:
    while True:
        print("Encoder time: ", read_encoder())
        pass
except KeyboardInterrupt:
    GPIO.cleanup()
