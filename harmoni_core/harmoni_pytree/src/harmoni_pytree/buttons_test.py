"""Module importation"""
import serial

"""Opening of the serial port"""
arduino = serial.Serial("/dev/ttyUSB0",timeout=1)

print(arduino.readlines())

