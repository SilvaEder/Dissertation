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


    ## DISCRETIZE USING TUSTIN (MANUAL IMPLEMENTATION)
    def tustin_discretization(Ac, Bc, t):
        Ad = np.linalg.inv(np.eye(Ac.shape[0]) - (t / 2) * Ac).dot(np.eye(Ac.shape[0]) + (t / 2) * Ac)
        Bd = np.linalg.inv(np.eye(Ac.shape[0]) - (t / 2) * Ac).dot(t * Bc)
        return Ad, Bd


    ## DEFINING SYSTEM CONSTANTSa
    m_c = 0.330  # Cart mass (kg)
    m_p = 0.179  # Pendulum mass (kg)
    mu_c = 0.004  # Cart friction coefficient (N/m/s)
    mu_p = 0.001  # Pendulum friction coefficient (N/m/s)
    ell = 0.463  # Pendulum length (m)
    g = 9.810  # Acceleration due to gravity (m/s^2)
    jmath = 0.013  # Rotary inertia of the pendulum rod (kg m^2)

    a22 = -mu_c * (jmath + m_p * ell ** 2) / ((m_c + m_p) * (jmath + m_p * ell ** 2) - ((m_p * ell) ** 2))
    a23 = (m_p ** 2 * g * ell ** 2) / ((m_c + m_p) * (jmath + m_p * ell ** 2) - ((m_p * ell) ** 2))
    a24 = (mu_p * m_p * ell / ((m_c + m_p) * (jmath + m_p * ell ** 2) - ((m_p * ell) ** 2)))
    a42 = -(mu_c * m_p * ell) / ((m_c + m_p) * (jmath + m_p * ell ** 2) - ((m_p * ell) ** 2))
    a43 = ((m_c + m_p) * g * m_p * ell) / ((m_c + m_p) * (jmath + m_p * ell ** 2) - ((m_p * ell) ** 2))
    a44 = (-(m_c + m_p) * mu_p) / ((m_c + m_p) * (jmath + m_p * ell ** 2) - ((m_p * ell) ** 2))

    Ac = np.array([[0, 1, 0, 0], [0, a22, a23, a24], [0, 0, 0, 1], [0, a42, a43, a44]])
    b21 = (jmath + m_p * ell ** 2) / ((m_c + m_p) * (jmath + m_p * ell ** 2) - ((m_p * ell) ** 2))
    b41 = -(m_p * ell) / ((m_c + m_p) * (jmath + m_p * ell ** 2) - ((m_p * ell) ** 2))
    Bc = np.array([[0], [b21], [0], [b41]])
    Ad, Bd = tustin_discretization(Ac, Bc, t)
    Q = np.diag([1, 1e-06, 100, 1e-06])
    R = np.array([[0.01]])

    c = ModbusClient(host="192.168.15.150", port=502, auto_open=True, auto_close=True)

    simulation_time = input('[INPUT] Digite o Tempo de Simulação em Segundos: ')
    simulation_time = float(simulation_time)

    # Define linspace and initial conditions

    u = []
    j = []
    i = 0
    X1 = []
    X2 = []
    X3 = []
    X4 = []
    ACC = []
    acc = 0
    action = np.linspace(-1500, 1500, num=3001)
    startTime = time.time()

    x = []
    encoder_position = c.read_holding_registers(0, 4)
    x1=(float(struct.unpack('>h', struct.pack('>H', encoder_position[2]))[0]))
    x3=(float(struct.unpack('>h', struct.pack('>H', encoder_position[0]))[0]))

    print("[INFO] Posiciona a Haste proximo a 0°")

    while x3 < -30.0 or x3 > 30.0:
        encoder_position = c.read_holding_registers(0, 4)
        x3=(float(struct.unpack('>h', struct.pack('>H', encoder_position[0]))[0]))
        time.sleep(t)
    print("[INFO] Teste Iniciado")

    x.append(np.array([[x1], [0], [x3], [0]]))

    while elapsedTime < simulation_time:
        encoder_position = c.read_holding_registers(0, 4)
        x1=(float(struct.unpack('>h', struct.pack('>H', encoder_position[2]))[0]))
        x3=(float(struct.unpack('>h', struct.pack('>H', encoder_position[0]))[0]))

        X = []
        Useq = []

        N = 0

        X1.append(x1)
        X2.append(0)
        X3.append(x3)
        X4.append(0)

        X.append(x[i])

        while N < 3:
            J_valuep = 9000000
            for k in range(len(action)):
                x_sim = Ad.dot(X[N]) + Bd.dot(action[k])
                J_value = x_sim.transpose().dot(Q).dot(x_sim) + np.array(action[k]).transpose().dot(R).dot(
                    np.array(action[k]))
                if J_value <= J_valuep:
                    U = action[k]
                    J_valuep = J_value

            Useq.append(U)
            X.append(Ad.dot(x[i]) + Bd.dot(Useq[N]))
            N += 1

        if Useq[0] > 1500:
            u_inp = 1500
        elif Useq[0] < -1500:
            u_inp = -1500
        else:
            u_inp = Useq[0]
        c.write_multiple_registers(10, [struct.unpack('>H', struct.pack('>h', int(u_inp)))[0]])
        x.append(np.array([[x1], [0], [x3], [0]]))
        u.append(Useq[0])
        i += 1
        #acc = x[i].transpose().dot(Q).dot(x[i]) + np.array(Useq[0]).transpose().dot(R).dot(np.array(Useq[0]))
        #ACC.append(acc[0][0])

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

    print("[INFO] Fim do Teste")
    plt.figure(figsize=(12, 8))

    plt.subplot(5, 1, 1)
    plt.plot(range(len(X1)), X1, label=r'$x_{1}$', color="#90EE90")
    plt.ylabel(r'$x_{1}$')
    sns.despine()
    plt.legend()
    plt.grid(True)

    plt.subplot(5, 1, 2)
    plt.plot(range(len(X1)), X2, label=r'$x_{2}$', color="#ADD8E6")
    plt.ylabel(r'$x_{2}$')
    sns.despine()
    plt.legend()
    plt.grid(True)

    plt.subplot(5, 1, 3)
    plt.plot(range(len(X1)), X3, label=r'$x_{3}$', color='#FFA07A')
    plt.ylabel(r'$x_{3}$')
    sns.despine()
    plt.legend()
    plt.grid(True)

    plt.subplot(5, 1, 4)
    plt.plot(range(len(X1)), X4, label=r'$x_{4}$', color='#CC79A7')
    plt.ylabel(r'$x_{4}$')
    sns.despine()
    plt.legend()
    plt.grid(True)

    plt.subplot(5, 1, 5)
    plt.step(range(len(u)), u, color='#E69F00', label='Control input')
    plt.xlabel('Steps')
    plt.ylabel(r'$u$')
    plt.grid(True)
    plt.legend()
    sns.despine()

    plt.tight_layout()
    plt.show()

