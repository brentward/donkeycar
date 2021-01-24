#!/usr/bin/env python3
"""
"""

import time
import struct
import donkeycar as dk

try:
    import serial
except ImportError:
    print("PySerial not found.  Please install: pip install pyserial")


class TeensyUltrasonic:
    '''
    Driver to read signals from Teensy over serial and convert into steering and throttle outputs and
    ultrasonic distance ranging
    Output range: 0.0 to 500.0 for distance in cm
    '''

    def __init__(self, port, baud_rate=115200, debug=True):
        # standard variables
        self.port = port
        self.baud_rate = baud_rate
        self.cm = 0.0
        self.debug = debug
        self.serial = None
        self.on = False

        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=0.01)
        except serial.SerialTimeoutException:
            print("Serial connection timed out!")
        except serial.SerialException:
            print("Serial port not found!  Please enable: sudo raspi-config")
        else:
            self.on = True

    def shutdown(self):
        print("Shutting down serial connection to Teensy Controller on {}".format(self.port))
        self.on = False
        time.sleep(1)
        self.serial.flush()
        try:
            self.serial.close()
        except Exception:
            pass

    def read_serial(self):
        '''
        Read the rc controller value from serail. Map the value into
        distance

        Format is an struct containing a single unsigned short integer
        '''
        try:
            data_rx = b''
            while len(data_rx) != 2:
                data_rx = self.serial.read(2)
        except serial.SerialException as e:
            print('Teensy Controller Serial Exception: {}'.format(e))
        else:
            try:
                data = struct.unpack('H', data_rx)
            except struct.error as e:
                print('Teensy Controller: invalid bytes for struct: {}'.format(e))
            else:
                range_pw = data[0]

                if self.debug:
                    print("range_pw = {}".format(range_pw))

                if range_pw != 0:
                    self.cm = round(range_pw / 52.0, 2)

                if self.debug:
                    print("cm = {}".format(self.cm))

                time.sleep(0.01)

    def update(self):
        # delay on startup to avoid crashing
        print("Teensy Controller: warming serial port...")
        time.sleep(3)
        while not self.on:
            time.sleep(1)
        self.serial.reset_input_buffer()
        while self.on:
            self.read_serial()

    def run(self, img_arr=None):
        return self.run_threaded()

    def run_threaded(self, img_arr=None):
        return self.cm
