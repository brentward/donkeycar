#!/usr/bin/env python3
"""
Script to read signal from Arduino and and return it as distance in cm
"""

import serial
import time

class SerialUltrasonic:
    def __init__(self, port='/dev/ttyTHS1', baudrate=9600):
        print("Starting Serial Ultrasonic")

        self.port = port
        self.baudrate = baudrate
        self.cm = 0.0
        self.serial = serial.Serial(self.port, self.baudrate, timeout=1) #Serial port - Jetson Nano UART GPIO 8 & 10: '/dev/ttyTHS1'
        # delay on startup to avoid crashing
        print("Warming Serial Ultrasonic...")
        time.sleep(1)
        self.on = True

    def update(self):
        
        while self.on:
            try:
                self.serial.write(b'\n')
                serial_response = self.serial.readline().decode().strip('\n').strip('\r')
            except serial.SerialException as e:
                print('Serial Exception: {}'.format(e))
            else:
                self.cm = float(serial_response)
            time.sleep(0.01)

    def run(self, img_arr=None):
        return self.run_threaded()

    def run_threaded(self, img_arr=None):
        # print("Range: {}".format(self.cm))
        return self.cm

    def shutdown(self):
        print("Stopping Serial Ultrasonic")
        self.on = False
        time.sleep(1)
        self.serial.flush()
        self.serial.close()