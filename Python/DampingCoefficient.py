##IMPORTING LIBRARIES
import time
import numpy as np
import pandas as pd
import seaborn as sns
from datetime import datetime
from numpy.linalg import inv
import matplotlib.pyplot as plt
from pyModbusTCP.client import ModbusClient

# TCP AUTO CONNECT ON MODBUS REQUEST, CLOSE AFTER IT
c = ModbusClient(host="192.168.15.150", port=502,auto_open=True, auto_close=True)

# DECLARATION OF LOCAL VARIABLES
# SAMPLE TIME
t = .05
Time = 0
NewTime = t
PrevTime = 0
TimeError = 0
ElapsedTime = 0

Error = []
RealTime = []
simulateTime = []

## INITIALIZING THE SYSTEM VARIABLES
elapsedTime = 0
servo_position = 0
encoder_position = 0

x1 = []
x3 = []
u = []
center = 6000 / 2
delta = 6000 / 2
r = center - delta

## INPUT THE SYSTEM VARIABLES
SimulateTime = input('[INPUT] Digite o Tempo de Simulação em Segundos\n')
# CONVERT INPUT TO INTEGER
SimulateTime = int(SimulateTime)

startTime = time.time()
while elapsedTime < SimulateTime:
    encoder_position = c.read_holding_registers(0, 6)
    x1.append(float(encoder_position[0]))
    x3.append(float(encoder_position[2]))
    u.append(float(encoder_position[4]/10))

    if r <= center and encoder_position[2] > (center - delta):
        c.write_multiple_registers(10, [int(r)])
    elif r <= center and encoder_position[2] <= (center - delta):
        r = center + delta
        c.write_multiple_registers(10, [int(r)])
    elif r >= center and encoder_position[2] < (center + delta):
        c.write_multiple_registers(10, [int(r)])
    elif r >= center and encoder_position[2] >= (center + delta):
        delta -= 100
        r = center - delta
        c.write_multiple_registers(10, [int(r)])
    # SAMPLE TIME
    time.sleep(NewTime)
    Time += t

    ElapsedTime = time.time() - startTime
    TimeError = 2 * (Time - ElapsedTime)
    NewTime += TimeError

    if ((NewTime) < 0):
        NewTime = 0.00001
    Error.append(ElapsedTime - Time)
    RealTime.append(ElapsedTime)
    simulateTime.append(Time)

    elapsedTime = time.time() - startTime
elapsedTime = round(time.time() - startTime, 2)
print(f'[INFO] Tempo do experimento {elapsedTime}s')

timestamp = datetime.now().strftime("Data_%d_%b_%y_%H-%M.csv")
timesfigure = datetime.now().strftime("Figure_%d_%b_%y_%H-%M.png")

dataframe = pd.DataFrame([x1,x3,u,Error,RealTime, simulateTime])
dataframe.to_csv(timestamp, index=False)

plt.figure(figsize=(10,8))
n=np.arange(0,len(x1),1)
plt.subplot(311)
plt.plot(n,x1, color='r', label='Encoder')
plt.title("Ajustes dos Dados Aquisitados")
plt.ylabel("Posição Angular")
plt.xlabel("Tamanho")
plt.grid(True)
plt.legend()
sns.despine()
plt.subplot(312)
plt.plot(n,x3, color='b', label='Encoder')
plt.ylabel("Posição Angular")
plt.xlabel("Tamanho")
plt.grid(True)
plt.legend()
sns.despine()
plt.subplot(313)
plt.plot(n,u, color='#E69F00', label='Encoder')
plt.ylabel("Torque %")
plt.xlabel("Tamanho")
plt.grid(True)
plt.legend()
sns.despine()
plt.savefig(timesfigure)
plt.show()
