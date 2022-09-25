import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from scipy import linalg

data = genfromtxt('Radar_Lidar_Data1.csv', delimiter=',')
data = np.nan_to_num(data)
data_split = np.array(data[1:,1:])

# defining time rate of change
d_t = 0.1

F = np.array([[1,0,d_t,0],
    [0,1,0,d_t],
    [0,0,1,0],
    [0,0,0,1]])
u = 0

B = np.array([[(d_t**2)/2],[(d_t**2)/2],[d_t],[d_t]])

P = np.array([[1,0,0,0],[0,1,0,0],[0,0,1000,0],[0,0,0,1000]])

R_l = np.array([[[0.0025, 0],
                [0, 0.0025]]])

R_r = np.array([[0.09, 0, 0],
                [0, 0.005, 0],
                [0, 0, 0.09]])

Q = np.array([[(d_t**2)/4,0, (d_t**3)/2,0],
            [ 0,(d_t**2)/4,0,(d_t**3)/2 ],
            [ (d_t**3)/2,0,(d_t**2)/4,0 ],
            [ 0,(d_t**3)/2,0,(d_t**2)/4]])

H = np.array([[1, 0, 0, 0],
     [0, 1, 0, 0]])

I = np.identity(4,dtype='int')

if data_split[0,0] == 1 :
    x = np.array([[data_split[1,2]], [data_split[1,3]], [0], [0]])
else :
    x = np.array([[data_split[1,2]], [data_split[1,3]], [data_split[1,4]], [0]])

rows, cols = data_split.shape
EKF_Path = np.zeros((rows,2))
Radar_Measurement = []
Lidar_Measurement = []
Radar_Measurement_Cart = []
for i in range(rows):

    if data_split[i,0] == 2:

        x = np.matmul(F , x)  + (B * u)
        # print("B * u = ", B*u)
        P = np.matmul(np.matmul(F , P) , np.transpose(F)) + Q

        # measurement update
        Z = data_split[i,1:4]
        Z = np.reshape(Z,(3,1))
        # print(np.transpose(Z))
        X = Z[0]*np.cos(Z[1])
        Y = Z[0]*np.sin(Z[1])
        VX = Z[2]*np.cos(Z[1])
        VY = Z[2]*np.sin(Z[1])

        c1 = X**2 + Y**2
        c2 = np.sqrt(c1)
        c3 = np.multiply(c1,c2)

        if ((c1 == 0) or (c2 == 0) or (c3 == 0)) :
            H_Jac = np.array([[0, 0, 0, 0],
                     [0, 0, 0, 0],
                     [0, 0, 0, 0]],dtype=float)
        else :
            H_Jac = np.array([[X/c2, Y/c2, 0, 0],
                    [-Y/c1, X/c1, 0, 0],
                    [(Y*(VX*Y-VY*X))/c3, (X*(X*VY-Y*VX))/c3, X/c2, Y/c2]],dtype=float)

        Z_Car = np.array([X,Y, VX, VY],dtype=float)
        y = Z - np.matmul(H_Jac,Z_Car)
        mul_hjac_p = np.matmul(H_Jac,P)
        S =  np.matmul(mul_hjac_p , np.transpose(H_Jac)) + R_r
        multiply_p_hjac = np.matmul(P,np.transpose(H_Jac))
        K = np.matmul(multiply_p_hjac , np.linalg.inv(S))
        x = Z_Car + np.matmul(K , y)
        mul_I_K_H_Jac = I - np.matmul(K , H_Jac)
        P = np.matmul(mul_I_K_H_Jac, P)
        x = x.reshape(4,1)
        EKF_Path[i,0] = x[0]
        EKF_Path[i,1] = x[1]
        Radar_Measurement.append(data_split[i,1:4])
    else : 
        x = np.matmul(F , x)  + (B * u)
        P = np.matmul(np.matmul(F , P) , np.transpose(F)) + Q
        Z = data_split[i,1:3]
        Z = np.reshape(Z,(2,1))
        y = Z - np.matmul(H,x)
        mul_h_p = np.matmul(H,P)
        S =  np.matmul(mul_h_p , np.transpose(H)) + R_l

        multiply_p_transh = np.matmul(P,np.transpose(H))
        K = np.matmul(multiply_p_transh , np.linalg.inv(S))
        
        x = x + np.matmul(K , y)
        x = x.reshape(4,1)

        mul_I_K_H = I - np.matmul(K , H)
        P = np.matmul(mul_I_K_H , P)
        EKF_Path[i,0] = x[0]
        EKF_Path[i,1] = x[1]
        Lidar_Measurement.append(data_split[i,1:3])

Radar_Measurement = np.squeeze(Radar_Measurement)
Lidar_Measurement = np.squeeze(Lidar_Measurement)

for i in range(Radar_Measurement.shape[0]):
    Radar_Measurement_Cart.append([Radar_Measurement[i,0]*np.cos(Radar_Measurement[i,1]),Radar_Measurement[i,0]*np.sin(Radar_Measurement[i,1])])

Radar_Measurement_Cart = np.squeeze(Radar_Measurement_Cart)

plt.plot(data_split[:,5],data_split[:,6],color='black')
plt.scatter(EKF_Path[:,0],EKF_Path[:,1],color='red')
plt.scatter(Lidar_Measurement[:,0],Lidar_Measurement[:,1],color='blue')
plt.scatter(Radar_Measurement_Cart[:,0],Radar_Measurement_Cart[:,1],color='green')
plt.legend(['Actual Readings','EKF Localization','Lidar Data','Radar Data'])
plt.show()





