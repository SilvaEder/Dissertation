########################################################################################
# IMPORTING LIBRARIES
########################################################################################
import time
import struct
import numpy as np
import pandas as pd
import seaborn as sns
from mpi4py import MPI
from itertools import count
from numpy.linalg import inv
from datetime import datetime
import matplotlib.pyplot as plt
from pyModbusTCP.client import ModbusClient
from matplotlib.animation import FuncAnimation

########################################################################################
# START MESSAGE PASSING INTERFACE
########################################################################################
comm = MPI.COMM_WORLD
size = comm.Get_size()
rank = comm.Get_rank()

print('Rank:', rank, ', Inicializado')
comm.Barrier()

if __name__ == "__main__":
    # SAMPLE TIME
    t = .05
    Time = 0
    NewTime = t
    PrevTime = 0
    TimeError = 0
    ElapsedTime = 0

    Error = []
    RealTime = []
    SimulateTime = []

    elapsedTime = 0
    servo_position = 0
    encoder_position = 0

    if rank == 0:
        # Processo 0: envia o tempo de simulação e os valores
        # TCP AUTO CONNECT ON MODBUS REQUEST, CLOSE AFTER IT
        c = ModbusClient(host="192.168.15.150", port=502, auto_open=True, auto_close=True)

        simulation_time = input('[INPUT] Digite o Tempo de Simulação em Segundos: ')
        simulation_time = float(simulation_time)
        # Envia simulation_time para o processo 1
        comm.send(simulation_time, dest=1)
        # CONVERT INPUT TO INTEGER
        simulation_time = int(simulation_time)

        startTime = time.time()
        while elapsedTime < simulation_time:
            encoder_position = c.read_holding_registers(0, 4)
            x1=(float(struct.unpack('>h', struct.pack('>H', encoder_position[2]))[0]))
            x3=(float(struct.unpack('>h', struct.pack('>H', encoder_position[0]))[0]))

            print(f'rank 0: ',x3)
            comm.send(x3, dest=1)
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
                SimulateTime.append(Time)

            elapsedTime = time.time() - startTime
        elapsedTime = round(time.time() - startTime, 2)


    else:
        # Processo 1: recebe os dados e atualiza o gráfico
        simulation_time = comm.recv(source=0)  # recebe o tempo de simulação
        x_vals = []
        y_vals = []
        counter = count()  # contador para o eixo x
        setpoint = 0

        def animate(i):
            plt.cla()
            plt.xlim(0, int(simulation_time / t))
            plt.ylim(-1024, 1024)

            # Se houver mensagem, recebe o valor e atualiza os vetores
            if comm.Iprobe(source=0):
                x3 = comm.recv(source=0)
                x_vals.append(next(counter))
                y_vals.append(x3)
                print(f'rank 1',x3)

            plt.plot(x_vals, y_vals, "#0072B2", label='Encoder')
            plt.axhline(y=setpoint, linewidth=2, label='Setpoint', color='#DC143C')

            sns.despine()
            plt.grid(True)
            plt.legend(loc='upper left')
            plt.title("Angular Position")
            plt.ylabel("Angular Position")
            plt.xlabel("Sample number")

        ani = FuncAnimation(plt.gcf(), animate, interval=30)
        plt.tight_layout()
        plt.show()

