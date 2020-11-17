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

T = 0.05

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')


# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw


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
                
        #Troca de estado
        if abs(error) < 1:
            Int = 0
            msg.angular.z = 0
            kp = 1
            ki = 0.03
            kd = 0.04 
            state = 1
            
        
    elif state == 1:    
        setpoint = 0.5
        scan_len = len(scan.ranges)
        if scan_len > 0:
            read = min(scan.ranges[scan_len-10 : scan_len+10])

            error = abs(setpoint - read)
            P = kp*error
            I = I + error * ki
            D = (error - old_error)*kd

            control = P + I + D
            old_error = error
            print(control)
            if control > 1:
                control = 1
            elif control < -1:
                control = -1
        else:
            control = 0 
        
        msg = Twist()
        msg.linear.x = control
        print("State: ", state)
        print(error)
        if (read < 0.5):
            msg.linear.x = 0
            state = 0
            print("State: ", state)
          
    pub.publish(msg)
    print(msg.linear.x)
    
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(T), timerCallBack)

rospy.spin()