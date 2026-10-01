########################################################################################
## IMPORTING LIBRARIES
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
## START MESSAGE PASSING INTERFACE
########################################################################################
comm = MPI.COMM_WORLD
size = comm.Get_size()
rank = comm.Get_rank()

if __name__ == "__main__":
    ########################################################################################
    ## INITIALIZE LOCAL VARIABLES
    ########################################################################################
    if rank == 0:
        ## SAMPLE TIME VARIABLES
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

        ## FILTER VARIABLES
        Phi = np.array([[1, t], [0, 1]])
        Gamma = np.array([[0], [t]])
        H = np.array([[1, 0], [0, 0]])
        Q = np.array([[10, 0], [0, 10]])
        R = np.array([[.1, 0], [0, 1]])
        update = [np.array([[0], [0]]), np.array([[0], [0]]), np.array([[0, 0], [0, 0]]), np.array([[0, 0], [0, 0]])]
        x_pri1 = [0] * 3
        x_pri3 = [0] * 3
        acel_pri1 = [0] * 20
        acel_pri3 = [0] * 20

    comm.Barrier()
    ########################################################################################
    ## LOCAL FUNCTIONS
    ########################################################################################
    if rank == 0:
        def KalmanFilter(x1, x3, x1_pos, x3_pos, P1_pos, P3_pos):
            """
            Kalman Filter Velocity Estimation
            @author1:    Eder H. N. Silva
            @author2:    Gilmar
            Date:       March 10, 2024
            """
            """
            MOVING AVERAGE
            """
            x_pri1.pop(0)
            x_pri3.pop(0)
            acel_pri1.pop(0)
            acel_pri3.pop(0)
            x_pri1.append(x1)
            x_pri3.append(x3)
            acel_pri1.append(((x_pri1[2] - x_pri1[1]) - (x_pri1[1] - x_pri1[0])) * (1 / t * 2))
            acel_pri3.append(((x_pri3[2] - x_pri3[1]) - (x_pri3[1] - x_pri3[0])) * (1 / t * 2))
            acel_avr1 = (sum(acel_pri1) / 20)
            acel_avr3 = (sum(acel_pri3) / 20)
            """
            PREDICTION STEP
            """
            ## A PRIORI STATE ESTIMATE
            x1_pri = Phi.dot(x1_pos) + Gamma.dot(acel_avr1)
            x3_pri = Phi.dot(x3_pos) + Gamma.dot(acel_avr3)
            ## A PRIORI COVARIANCE MATRIX ESTIMATE
            P1_pri = (Phi.dot(P1_pos)).dot(Phi.transpose()) + Q
            P3_pri = (Phi.dot(P3_pos)).dot(Phi.transpose()) + Q
            """
            UPDATE STEP
            """
            ## KALMAN GAIN
            K1 = (P1_pri.dot(H.transpose())) * (np.linalg.inv(((H).dot(P1_pri)).dot(H.transpose()) + R))
            K3 = (P3_pri.dot(H.transpose())) * (np.linalg.inv(((H).dot(P3_pri)).dot(H.transpose()) + R))
            # A POSTERIORI STATE ESTIMATE
            x1_pos = x1_pri + K1.dot(([[x1], [0]] - H.dot(x1_pri)))
            x3_pos = x3_pri + K3.dot(([[x3], [0]] - H.dot(x3_pri)))
            # A POSTERIORI COVARIANCE MATRIX ESTIMATE
            P1_pos = P1_pri - K1.dot(H.dot(P3_pri))
            P3_pos = P3_pri - K3.dot(H.dot(P3_pri))
            return x1_pos, x3_pos, P1_pos, P3_pos

    comm.Barrier()
    ########################################################################################
    ## GLOBAL VARIABLE AND FUNCTIONS
    ########################################################################################

    ## DISCRETIZE USING TUSTIN
    def tustin_discretization(Ac, Bc, t):
        Ad = np.linalg.inv(np.eye(Ac.shape[0]) - (t / 2) * Ac).dot(np.eye(Ac.shape[0]) + (t / 2) * Ac)
        Bd = np.linalg.inv(np.eye(Ac.shape[0]) - (t / 2) * Ac).dot(t * Bc)
        return Ad, Bd

    ## DEFINING SYSTEM CONSTANTSa
    m_c = 0.360  # Cart mass (kg)
    m_p = 0.179  # Pendulum mass (kg)
    mu_c = 0.004  # Cart friction coefficient (N/m/s)
    mu_p = 0.001  # Pendulum friction coefficient (N/m/s)
    ell = 0.463  # Pendulum length (m)
    g = 9.810  # Acceleration due to gravity (m/s^2)
    jmath = 0.013  # Rotary inertia of the pendulum rod (kg m^2)

    ## SAMPLE TIME
    t = .05

    ## STATE MATRICES
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

    action = np.linspace(-2000, 2000, 4001)
    p = int(len(action) / (size - 1))

    comm.Barrier()
    ########################################################################################
    ## INPUT THE SYSTEM VARIABLES
    ########################################################################################
    if rank == 0:
        c = ModbusClient(host="192.168.15.150", port=502, auto_open=True, auto_close=True)

        print(f'[INFO] Posicionamento')
        c.write_multiple_registers(10, [int(0)])
        time.sleep(1.5)
        simulation_time = input('[INPUT] Digite o Tempo de Simulação em Segundos: ')
        simulation_time = float(simulation_time)

    comm.Barrier()
    ########################################################################################
    ## THE PROGAM
    ########################################################################################
    if rank == 0:
        ## INITIALIZES LOCAL VARIABLES
        x = [];u = [];j = [];ACC = []
        X1 = [];X2 = [];X3 = [];X4 = []
        X1_REAL = [];X3_REAL = []
        acc = 0

        print("[INFO] Posiciona a Haste proximo a 0°")
        encoder_position = c.read_holding_registers(0, 4)
        x3 = (float(struct.unpack('>h', struct.pack('>H', encoder_position[0]))[0]) * (360 / 1024))

        while x3 < -3 or x3 > 3:
            encoder_position = c.read_holding_registers(0, 4)
            x3 = (float(struct.unpack('>h', struct.pack('>H', encoder_position[0]))[0]) * (360 / 1024))

        print("[INFO] Teste Iniciado")
        startTime = time.time()
        while elapsedTime < simulation_time:
            ## READING THE STATES OF THE CLP
            encoder_position = c.read_holding_registers(0, 4)
            x1 = (float(struct.unpack('>h', struct.pack('>H', encoder_position[2]))[0]) * (.5 / 6000))
            x3 = (float(struct.unpack('>h', struct.pack('>H', encoder_position[0]))[0]) * (360 / 1024))

            ## ESTIMATE OF STATES
            update = KalmanFilter(x1,x3,update[0],update[1], update[2], update[3])

            ## STORES THE STATES
            X1.append(update[0][0][0])
            X2.append(update[0][1][0])
            X3.append(update[1][0][0])
            X4.append(update[1][1][0])
            X1_REAL.append(x1)
            X3_REAL.append(x3)

            ## CREATE STATE VECTOR
            #x = np.array([[update[0][0][0]],[update[0][1][0]],[update[1][0][0]],[update[1][1][0]]])
            x = np.array([[x1],[update[0][1][0]],[x3],[update[1][1][0]]])
            #x = np.array([[x1], [0], [x3], [0]])

            ## SENDS THE STATE VECTOR TO THE OTHER PROCESSES
            comm.send(x, dest=1)
            comm.send(x, dest=2)
            comm.send(x, dest=3)
            comm.send(x, dest=4)
            comm.send(x, dest=5)
            comm.send(x, dest=6)
            comm.send(x, dest=7)
            comm.send(x, dest=8)
            comm.send(x, dest=9)
            comm.send(x, dest=10)
            comm.send(x, dest=11)

            ## RECEIVES THE CALCULATION WITH THE LOCAL MINIMA OF THE OTHER PROCESSES
            df1 = comm.recv(source=1)
            df2 = comm.recv(source=2)
            df3 = comm.recv(source=3)
            df4 = comm.recv(source=4)
            df5 = comm.recv(source=5)
            df6 = comm.recv(source=6)
            df7 = comm.recv(source=7)
            df8 = comm.recv(source=8)
            df9 = comm.recv(source=9)
            df10 = comm.recv(source=10)
            df11 = comm.recv(source=11)

            ## SELECTS THE GLOBAL MINIMUM
            df = pd.concat([df1, df2, df3, df4, df5, df6, df7, df8, df9, df10, df11], ignore_index=True)
            df.columns = ['J1','J2','J3', 'u1', 'u2', 'u3']
            u_input = df.loc[df['J1'].idxmin()]['u1']

            print(f'u = ',u_input)

            ## SENDS THE OPTIMAL CONTROL TO THE PLC
            c.write_multiple_registers(10, [struct.unpack('>H', struct.pack('>h', int(u_input)))[0]])
            u.append(u_input)
            # acc = x[i].transpose().dot(Q).dot(x[i]) + np.array(Useq[0]).transpose().dot(R).dot(np.array(Useq[0]))
            # ACC.append(acc[0][0])

            ## SAMPLE TIME CORRECTION
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

        ########################################################################################
        ## CODE OUTPUTS
        ########################################################################################
        ## SAVE THE DATA
        timestamp = datetime.now().strftime("CSV/Data_%d_%b_%y_%H-%M.csv")
        timesfigure = datetime.now().strftime("Plot/Figure_%d_%b_%y_%H-%M.png")
        #dataframe = pd.DataFrame([X1,X2,X3,X4,X1_REAL,X3_REAL,u, Error, RealTime, SimulateTime])
        #dataframe.to_csv(timestamp, index=False)

        ## PLOTS THE GRAPH
        plt.figure(figsize=(12, 8))
        plt.subplot(5, 1, 1)
        plt.plot(range(len(X1)), X1, label=r'$x_{1}$', color="#90EE90")
        #plt.plot(range(len(X1)), X1_REAL, color='green', linestyle='--', label='Encoder')
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
        #plt.plot(range(len(X1)), X3_REAL, color='r', linestyle='--', label=r'Encoder')
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
        #plt.savefig(timesfigure)
        plt.tight_layout()
        plt.show()

    else:
        ## INITIALIZES LOCAL VARIABLES
        Q = np.diag([.1, 1e-9, 350,1e-9])
        R = np.array([[0.04]])

        while True:
            ## INITIALIZES LOCAL VARIABLES
            N = 0;U = 0
            J = [];X = [];Useq = []

            ## RECEIVES THE STATE VECTOR OF PROCESS 0
            X.append(comm.recv(source=0))

            ## STARTS THE MPC CALCULATION
            while N < 3:
                Jp = 1000000
                for k in range(((rank - 1) * p), ((rank) * p)):
                    x_sim = Ad.dot(X[N]) + Bd.dot(action[k])
                    J_value = (x_sim.transpose().dot(Q).dot(x_sim) + np.array(action[k]).transpose().dot(R).dot(
                        np.array(action[k])))[0][0]
                    if J_value <= Jp:
                        U = action[k]
                        Jp = J_value

                ## STORES THE OPTIMAL MPC VALUES
                Useq.append(U)
                J.append(Jp)

                ## UPDATE THE MODEL
                X.append(Ad.dot(X[N]) + Bd.dot(Useq[N]))
                N += 1

            ## CREATES THE DATAFRAME
            df = pd.DataFrame([J[0],J[1],J[2], Useq[0], Useq[1], Useq[2]])
            df = df.T
            ## SENDS THE DATAFRAME TO PROCESS 0
            comm.send(df, dest=0)