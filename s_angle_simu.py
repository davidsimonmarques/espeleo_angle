#!/usr/bin/env python
# license removed for brevity
#imports:________________________________
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import euler_from_quaternion, euler_matrix, quaternion_matrix
from sensor_msgs.msg import Imu
import numpy as np
from math import sqrt, acos 
#_________________________________________  
#Diagrama da posicao dos pontos de contato do espeleo_robo:
#    |p6| ____ |p1|             
#     /     X   \         
#  |p5|  y__|   |p2|         
#     \         /            
#    |p4| ____ |p3|             
# 
m = 6 #numero de pontos de contato
dx = 0.212 
dy = 0.33 #distancia entre p6 e p1 = distancia entre p4 e p3          
dy_m = 0.425 #distancia entre p5 e p2 - rodas mais afastadas      
dz = 0.135 #altura do centro de massa        
#-----------------------------------
#Coordenadas dos pontos de contato:
#-----------------------------------
#pi = [px, py, pz] em [metros]
p1 = np.array([ dx, -(dy/2)  , -dz])
p2 = np.array([  0, -(dy_m/2), -dz]) #roda do meio
p3 = np.array([-dx, -(dy/2)  , -dz])  
p4 = np.array([-dx,  (dy/2)  , -dz]) 
p5 = np.array([  0,  (dy_m/2), -dz]) #roda do meio
p6 = np.array([ dx,  (dy/2)  , -dz]) 

#----------------------------------
#Variaveis para receber o valor dos pontos multiplicados pela matriz de rotacao
p1_l = [0, 0, 0]
p2_l = [0, 0, 0]
p3_l = [0, 0, 0]
p4_l = [0, 0, 0]
p5_l = [0, 0, 0]
p6_l = [0, 0, 0]
p = [0, 0, 0, 0, 0, 0] #Esse vetor recebera em cada coluna um vetor pi_l (i = 1,2,...,6)
#----------------------------------
a = np.zeros((6, 3))
ang_final = np.zeros((6,1))
e = [0, 0, 0] #inicializacao da variavel que recebe os angulos de euler da imu do simulador
r = np.zeros((4,4)), #inicializacao da variavel que recebe o resultado da conversao de quaternio em matriz de rotacao
rot_matrix = np.zeros((3,3)) #inicializacao da variavel que recebe a matriz de rotacao reduzida para 3x3              
fg = np.array([0, 0, -1])#CORRIGIR!!!!!!!!!!!!!!!!!!!!
I = np.zeros((6,3))#inicializacao da variavel que recebe
Y = np.zeros((6,3))#inicializacao da variavel que recebe

def modulo(x):
    mod = 0
    soma = 0
    i = len(x)
    for k in range(i):
        soma += x[k]*x[k]
    mod = sqrt(soma)
    return mod

def min(x):
    m = 999999
    for i in range(len(x)):
        if x[i]< m:
            m = x[i]
    return m



def callback_imu(data):
    global quat, r, rot_matrix, e, I, Y, ang_final, fg
    global p1, p2, p3, p4, p5, p6, p1_l, p2_l, p3_l, p4_l, p5_l, p6_l, p_l, a, p

    e[0] = data.x
    e[1] = data.y
    e[2] = data.z
    r = np.array(euler_matrix(e[0], e[1], e[2]))#retorna matriz de rotacao 4x4
    rot_matrix = r[0:3, 0:3] #corta a matriz de rotacao em 3x3 e armazena em rot_matrix
    #multiplicando cada ponto pi(i = 1,2,...,6) pela matriz de rotacao, resultando em pontos pi_l(i = 1,2,...,6):
    p1_l = np.dot(rot_matrix, p1)
    p2_l = np.dot(rot_matrix, p2)
    p3_l = np.dot(rot_matrix, p3)
    p4_l = np.dot(rot_matrix, p4)
    p5_l = np.dot(rot_matrix, p5)
    p6_l = np.dot(rot_matrix, p6) 
    p = np.array([p1_l, p2_l, p3_l, p4_l, p5_l, p6_l])#coloca todos os pontos em um array
    for i in range(len(a)-1):
        a[i] = p[i+1]-p[i]
        print a[i]
    a[5] = p[0]-p[5]
    for i in range(len(a)):
        a[i] = a[i]/modulo(a[i])
    for i in range(len(a)-1):
        I[i] = p[i+1] - np.dot((np.square(a[i])), p[i+1])
    I[5] = p[0]- np.dot((np.square(a[i])), p[0])#CORRIGIR!!!!!!!!!!!!!!!!!!!!!!!!!!!    
    for i in range(len(Y)):
        Y[i] = np.arccos(np.dot(fg/modulo(fg), I[i]/modulo(I[i])))
    Y = np.degrees(Y)
    for i in range(len(Y)):
       ang_final[i] = min(Y[i])   

def listener():
    rospy.init_node('stability_angle', anonymous=True)
    rospy.Subscriber("/ros_eposmcd/gyro", Point, callback_imu)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print fg
        print "\n"
        print ang_final
        print "\n"
	rate.sleep()

            

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
       
   
