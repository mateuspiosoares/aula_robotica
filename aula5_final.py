#coordenadas -1.6747492596012297 1.5183799625393037

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


state = 0


kp = float(input("kp = "))
ki = float(input("ki = "))
kd = float(input("kd = "))
I = 0
setpoint = 0
process_var = 0
error = 0
old_error = 0
read = 0
Int = 0

T = 0

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')


matricula = [2019000063, 2017006353, 2017005795, 35132, 2017014453]


# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw

pinha = '2017005795'
pio = '2017006353'
barbara = '2017005795'
lucas = '35132'
miguel = '2017014453'
soma = 0
string = pinha + pio + barbara + lucas + miguel
div = len(string)
for i in range(div):
    soma = soma + int(string[i])
T = soma/div
print(T)

'''
def mediaSomaMatriculas(msg):
    media = 0
    for matricula in msg:
        somaMatricula = 0
        for i in str(matricula):
            somaMatricula += int(i)
            
        media += somaMatricula
        
    media = float(media)/len(msg)
    return media
    
frequencia = mediaSomaMatriculas(matricula)
T = 1.0/frequencia
print('Tempo: '+str(T))
'''

# CALLBACKS ---------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg


#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    global state
    global kp
    global ki
    global I
    global kd
    global process_var
    global setpoint
    global error
    global old_error
    global pub
    global read
    global T
    global Int
    
    if state == 0:
        scan_len = len(scan.ranges)
        
        if scan_len > 0:
            yaw = getAngle(odom)
            
            ind = scan.ranges.index(min(scan.ranges))
            inc = 2*math.pi / scan_len
            ang = (ind * inc * 180.0/math.pi) + yaw
            if ang > 180:
                ang -= 360
                
            error = (ang - yaw)
            
            if abs(error) > 180:
                if setpoint < 0:
                    error += 360 
                else:
                    error -= 360
                    
            print(ang, yaw, error)
            
            delta_e = error - old_error
            old_error = error
            
            P = kp*error
            Int += error*T
            I = Int * ki
            D = delta_e * kd
            
            control = P+I+D
            if control > 0.5:
                control = 0.5
            elif control < -0.5:
                control = -0.5
        else:
            control = 0
        msg = Twist()
        msg.angular.z = control
        pub.publish(msg)
        print("State: ", state)
        print("Erro:", abs(error))
        #Troca de estado
        if ( abs(error) < 1):
            Int = 0
            msg.angular.z = 0
            pub.publish(msg)
            '''
            kp = 1
            ki = 0.03
            kd = 0.04 
            '''
            state = 1
            
        
    elif state == 1:    
        setpoint = 0.5
        scan_len = len(scan.ranges)
        
        if scan_len > 0:
            read = min(scan.ranges[scan_len-10 : scan_len+10])
            print("Read: ", read)
            if read < 999:
                error = abs(setpoint - read)
                
                delta_e = error - old_error
                old_error = error
            
                P = kp*error
                Int += error*T
                I = Int * ki
                D = delta_e * kd
                
                '''
                P = kp*error
                I = Int + error * ki
                D = (error - old_error)*kd
                '''    
                control = P + I + D
                #old_error = error
                print("Control: ", control)
                if control > 1:
                    control = 1
                elif control < -1:
                    control = -1
                
                msg = Twist()
                msg.linear.x = control
                pub.publish(msg)
                print("State: ", state)
                print("Erro: ", error)
            else: 
                print("Entrou no infinito")
                state = 0
        else:
            control = 0 

        if (0 < error < 0.5):
            msg.linear.x = 0
            pub.publish(msg)
            print("State: ", state)
            state = 2
            
    elif state == 2:
        scan_len = len(scan.ranges)
        read = min(scan.ranges[scan_len-10 : scan_len+10])
        
        print ('Cheguei!')
        print('Read: ', read)
        msg = Twist()
        msg.angular.z = 0
        msg.linear.x = 0
        pub.publish(msg)
        
        if (read > 2):
            state = 0

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(T), timerCallBack)

rospy.spin()