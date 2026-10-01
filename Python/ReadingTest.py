##IMPORTING LIBRARIES
import time
import struct
import numpy as np
import pandas as pd
import seaborn as sns
from datetime import datetime
from numpy.linalg import inv
import matplotlib.pyplot as plt
from pyModbusTCP.client import ModbusClient

## INITIALIZING THE SYSTEM VARIABLES
elapsedTime = 0
servo_position = 0
encoder_position = 0
x1 = []
x3 = []

# TCP AUTO CONNECT ON MODBUS REQUEST, CLOSE AFTER IT
c = ModbusClient(host="192.168.15.150", port=502,auto_open=True, auto_close=True)

## INPUT THE SYSTEM VARIABLES
simulation_time = input('[INPUT] Digite o Tempo de Simulação em Segundos\n')
# CONVERT INPUT TO INTEGER
simulation_time = int(simulation_time)

startTime = time.time()
while elapsedTime < simulation_time:
    encoder_position = c.read_holding_registers(0, 4)
    x1.append(float(struct.unpack('>h', struct.pack('>H', encoder_position[2]))[0]))
    x3.append(float(struct.unpack('>h', struct.pack('>H', encoder_position[0]))[0]))
    elapsedTime = time.time() - startTime
elapsedTime = round(time.time() - startTime, 2)

print(f'[INFO] Tempo do experimento {elapsedTime}s')

timestamp = datetime.now().strftime("Data_%d_%b_%y_%H-%M.csv")
timesfigure = datetime.now().strftime("Figure_%d_%b_%y_%H-%M.png")

dataframe = pd.DataFrame([x1,x3])
dataframe.to_csv(timestamp, index=False)

plt.figure(figsize=(10,8))
n=np.arange(0,len(x1),1)
plt.subplot(211)
plt.plot(n,x1, color='r', label='Encoder')
plt.title("Ajustes dos Dados Aquisitados")
plt.ylabel("Posição Angular")
plt.xlabel("Tamanho")
plt.grid(True)
plt.legend()
sns.despine()
plt.subplot(212)
plt.plot(n,x3, color='b', label='Encoder')
plt.ylabel("Posição Angular")
plt.xlabel("Tamanho")
plt.grid(True)
plt.legend()
sns.despine()
plt.savefig(timesfigure)
plt.show()
