import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

print('teste')
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
    
    if state == 0:
        '''
        setpoint = 0.5
    
        scan_len = len(scan.ranges)
        if scan_len > 0:
            read = min(scan.ranges[scan_len-10 : scan_len+10])

            error = setpoint - read
            P = kp*error
            I = I + error * ki
            D = (error - old_error)*kd

            control = P + I + D
            old_error = error
        
            if control > 1:
                control = 1
            elif control < -1:
                control = -1
        else:
            control = 0 
        msg = Twist()
        msg.angular.z = control
        
        '''
        msg = Twist()
        msg.angular.z = 0.5
        scan_len = len(scan.ranges)
        if scan_len > 0:
            read = min(scan.ranges[scan_len-10 : scan_len+10])
            print(read)
            
        if (0 < read < 100):
            msg.angular.z = 0
            print("State: ", state)
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
        if (read>100):
            msg.linear.x = 0
            state = 0
            print("State: ", state)
          
    pub.publish(msg)
    print(msg.linear.x)
    
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()