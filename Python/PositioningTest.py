##IMPORTING LIBRARIES
import time
import struct
import numpy as np
import pandas as pd
import seaborn as sns
from numpy.linalg import inv
import matplotlib.pyplot as plt
from pyModbusTCP.client import ModbusClient

## INITIALIZING THE SYSTEM VARIABLES
elapsedTime = 0
servo_position = 0
encoder_position = 0

# TCP AUTO CONNECT ON MODBUS REQUEST, CLOSE AFTER IT
c = ModbusClient(host="192.168.15.150", port=502,auto_open=True, auto_close=True)

## INPUT THE SYSTEM VARIABLES
servo_position = input('[INPUT] uma posição: ')
c.write_multiple_registers(10, [struct.unpack('>H', struct.pack('>h', int(servo_position)))[0]])

servo_position = input('[INPUT] uma posição: ')
c.write_multiple_registers(10, [struct.unpack('>H', struct.pack('>h', int(servo_position)))[0]])

servo_position = input('[INPUT] uma posição: ')
c.write_multiple_registers(10, [struct.unpack('>H', struct.pack('>h', int(servo_position)))[0]])

servo_position = input('[INPUT] uma posição: ')
c.write_multiple_registers(10, [struct.unpack('>H', struct.pack('>h', int(servo_position)))[0]])

servo_position = input('[INPUT] uma posição: ')
c.write_multiple_registers(10, [struct.unpack('>H', struct.pack('>h', int(servo_position)))[0]])

print("[INFO] FIM")