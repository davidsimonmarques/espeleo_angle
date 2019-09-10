#!/usr/bin/env python
# license removed for brevity
#imports:________________________________
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String, Float32, Bool, Float32MultiArray
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import euler_from_quaternion, euler_from_matrix, quaternion_matrix 
from sensor_msgs.msg import Imu
import numpy as np
from math import sqrt, acos 
#_________________________________________  
#Diagrama da posicao dos pontos de contato do espeleo_robo:
#    (p6) ____ (p1)             
#     |     X   |         
#  (p5)  y__|   (p2)       
#     |         |            
#   (p4) ____ (p3)             
# 
#m = 6 #numero de pontos de contato
#dx = 0.212 
#dy = 0.33 #distancia entre p6 e p1 = distancia entre p4 e p3          
#dy_m = 0.425 #distancia entre p5 e p2 - rodas mais afastadas      
#dz = 0.135 #altura do centro de massa 
m = rospy.get_param("m")
dx = rospy.get_param("dx")
dy = rospy.get_param("dy")
dy_m = rospy.get_param("dy_m")
dz = rospy.get_param("dz")
fg = np.array(rospy.get_param("fg"))       
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
p1_l = np.zeros(3)
p2_l = np.zeros(3)
p3_l = np.zeros(3)
p4_l = np.zeros(3)
p5_l = np.zeros(3)
p6_l = np.zeros(3)
p = np.zeros(6) #Esse vetor recebera em cada coluna um vetor pi_l (i = 1,2,...,6)
#----------------------------------
a = np.zeros((6, 3))
e = np.zeros(3)
ang_final = np.zeros((6,1))
quat = np.zeros(4) #inicializacao da variavel que recebe o quaternio da imu
r = np.zeros((4,4)) #inicializacao da variavel que recebe o resultado da conversao de quaternio em matriz de rotacao
rot_matrix = np.zeros((3,3)) #inicializacao da variavel que recebe a matriz de rotacao reduzida para 3x3              
#fg = np.array([0, 0, -1])
l = np.zeros((6,3))
Y = np.zeros(6)
sigma = np.zeros(6)
rpy_angles = Point()
min_angle = 0
flag = False
identidade = np.identity(3)
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
    global quat, fg, r, rot_matrix, l, Y, ang_final, identidade, sigma, min_angle, flag, rpy_angles
    global p1, p2, p3, p4, p5, p6, p1_l, p2_l, p3_l, p4_l, p5_l, p6_l, p_l, a, p
    identidade = np.identity(3)
    quat[0] = data.orientation.x 
    quat[1] = data.orientation.y
    quat[2] = data.orientation.z
    quat[3] = data.orientation.w 
    r = quaternion_matrix(quat)#retorna matriz de rotacao 4x4
    rot_matrix = r[0:3, 0:3] #corta a matriz de rotacao em 3x3 e armazena em rot_matrix
    #rot_matrix = np.identity(3) 
    rpy = euler_from_matrix(r, 'sxyz') #s significa eixos estaticos
    rpy_angles.x = np.degrees(rpy[0]) #roll em graus
    rpy_angles.y = np.degrees(rpy[1]) #pitch em graus
    rpy_angles.z = np.degrees(rpy[2]) #yaw em graus

    #multiplicando cada ponto pi(i = 1,2,...,6) pela matriz de rotacao, resultando em pontos pi_l(i = 1,2,...,6):
    p1_l = np.dot(rot_matrix,p1)
    p2_l = np.dot(rot_matrix,p2)
    p3_l = np.dot(rot_matrix,p3)
    p4_l = np.dot(rot_matrix,p4)
    p5_l = np.dot(rot_matrix,p5)
    p6_l = np.dot(rot_matrix,p6)

    p = np.array([p1_l, p2_l, p3_l, p4_l, p5_l, p6_l])#coloca todos os pontos em um array
    for i in range(len(a)-1):
        a[i] = p[i+1]-p[i]
    a[5] = p[0]-p[5]
    #print ("a nao normalizado: \n%s"%a)
    for i in range(len(a)):
        a[i] = a[i]/modulo(a[i])
    #print ("a normalizado: \n%s"%a)
    
    for i in range(len(l)-1):
        l[i] = np.dot((identidade - np.outer(a[i],np.transpose(a[i]))),p[i+1])
        #I[i] = np.dot((identidade-np.dot(a[i],a_t)),p[i+1])#erro multiplicacao e square e identidade
    l[5] = np.dot((identidade - np.outer(a[5],np.transpose(a[5]))),p[0])
    for i in range(len(sigma)):
            calc = np.dot(np.cross(l[i]/modulo(l[i]),fg/modulo(fg)),a[i])
            if calc < 0:
                sigma[i] = 1
            else:
                sigma[i] = -1
    for i in range(len(Y)):
        #Y[i] = np.arccos(-l[i][2]/modulo(l[i]))
        Y[i] = sigma[i]*np.arccos(np.dot(fg/modulo(fg), l[i]/modulo(l[i])))
    ang_final = np.rad2deg(Y)
    min_angle = min(ang_final)
    if min_angle < 10:
        flag = True
    else:
        flag = False
def procedure():
    rospy.init_node('stability_angle', anonymous=True)
    min_pub = rospy.Publisher('/min_angle', Float32, queue_size=10)
    angles_pub = rospy.Publisher('/angles', Float32MultiArray, queue_size=10)
    flag_pub = rospy.Publisher('/angle_flag', Bool, queue_size=10)
    rpy_pub = rospy.Publisher('/imu_rpy', Point, queue_size=10)
    rospy.Subscriber("/imu/data", Imu, callback_imu) #para celular:  rospy.Subscriber("/phone1/android/imu", Imu, callback_imu)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        a = Float32MultiArray()
        a.data = ang_final
        #a = [ang_final[0], ang_final[1], ang_final[2], ang_final[3], ang_final[4], ang_final[5]]   
        min_pub.publish(min_angle)   
        angles_pub.publish(a)
        if flag == True:
            flag_pub.publish(flag)
        rpy_pub.publish(rpy_angles)
        #print "\n"
        #print "-----------------------------------------------------"
        #print ("Quaternio IMU: %s"%quat)
        #print "-----------------------------------------------------"
        #print ("Matriz de rotacao: \n%s"%rot_matrix)
        #print "-----------------------------------------------------"
        #print ("P1: \n%s"%p1)
        #print "-----------------------------------------------------"
        #print ("a: \n%s"%a)
        #print "-----------------------------------------------------"        
        #print ("Matriz de rotacao multiplicada por p1: \n%s"%p1_l)
        #print "-----------------------------------------------------"
        #print ("p- Pontos multiplicados pela matriz de rotacao (linhas = pontos): \n%s"%p)
        #print "-----------------------------------------------------"  
        #print "Angulos (em graus):\n%s"%ang_final
        #print_diagrama(ang_final)
        #print "-----------------------------------------------------"  

	rate.sleep()

            

if __name__ == '__main__':
    try:
        procedure()
    except rospy.ROSInterruptException:
        pass
       
   
