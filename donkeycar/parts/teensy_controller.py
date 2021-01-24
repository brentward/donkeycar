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


class TeensyController:
    '''
    Driver to read signals from Teensy over i2c and convert into steering and throttle outputs and
    ultrasonic distance ranging
    Output range: -1.00 to 1.00 for steering and throttle and 0.0 to 500.0 for distance
    '''

    def __init__(self, cfg, debug=False):
        # standard variables
        self.port = cfg.TEENSY_CONTROLLER_PORT
        self.angle = 0.0
        self.throttle = 0.0
        self.cm = 0.0
        self.mode = 'user'
        self.recording = False
        self.DEAD_ZONE = cfg.JOYSTICK_DEADZONE
        self.debug = debug
        
        try:
            self.serial = serial.Serial(self.port, cfg.TEENSY_CONTROLLER_BAUDRATE, timeout=1)
        except serial.SerialTimeoutException:
            print("Serial connection timed out!")
        except serial.SerialException:
            print("Serial port not found!  Please enable: sudo raspi-config")

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
        Read the rc controller value from I2C bus. Map the value into
        steering and throttle

        Format is an array of 3 floats
        '''
        try:
            data_rx = self.serial.read_until(b'EOL')
        except serial.SerialException as e:
            print('Teensy Controller Serial Exception: {}'.format(e))
        else:
            try:
                data = struct.unpack('HHH', data_rx.strip(b'EOL'))
            except struct.error as e:
                print('Teensy Controller: invalid bytes for struct: {}'.format(e))
            else:
                angle_pwm = data[1]
                throttle_pwm = data[0]
                range_pwm = data[2]

                if self.debug:
                    print("angle_pwm = {}, throttle_pwm = {}, range_pwm = {}".format(angle_pwm, throttle_pwm, range_pwm))
                
                if angle_pwm > 2000:
                    angle_pwm = 2000
                if angle_pwm < 1000:
                    angle_pwm = 1000
                if throttle_pwm > 2000:
                    throttle_pwm = 2000
                if throttle_pwm < 1000:
                    throttle_pwm = 1000
                
                self.angle = dk.utils.map_range_float(angle_pwm, 1000, 2000, -1.0, 1.0)
                self.throttle = dk.utils.map_range_float(throttle_pwm, 1000, 2000, -1.0, 1.0)
                if range_pwm != 0:
                    self.cm = round(range_pwm / 52.0, 2)
                
                if self.debug:
                    print("angle = {}, throttle = {}, cm = {}".format(self.angle, self.throttle, self.cm))


                if self.throttle > self.DEAD_ZONE:
                    self.recording = True
                else:
                    self.recording = False

                time.sleep(0.01)

    def update(self):
        # delay on startup to avoid crashing
        print("Teensy Controller: warming serial port...")
        time.sleep(3)
        self.serial.reset_input_buffer()
        _garbage_data_rx = self.serial.read_until(b'EOL')
        while self.on:
            self.read_serial()

    def run(self, img_arr=None):
        return self.run_threaded()

    def run_threaded(self, img_arr=None):
        return self.angle, self.throttle, self.cm, self.mode, self.recording
