import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


kp = 1
ki = 0.01
kd = 1
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
    
    yaw = getAngle(odom)
    setpoint1 = 90
    error1 = (setpoint1 - yaw)
    
    if abs(error1) > 180:
        if setpoint1 < 0:
            error1 += 360 
        else:
            error1 -= 360
    P1 = kp*(error1)
    I1 =  ki*(error1)
    D1 = kd*(error1)
    control1 = P1+I1+D1
    
    setpoint2 = (2.683991025,1.887759912)
    position = odom.pose.pose.position
    dist = setpoint2[0] - position.x #math.sqrt((setpoint[0] - position.x)**2 + (setpoint[1] - position.y) **2)
    error2 = dist
    
    
    
    
    setpoint3 = 0.5
    
    scan_len = len(scan.ranges)
    if scan_len > 0:
        read = min(scan.ranges[scan_len-10 : scan_len+10])

        error3 = -(setpoint3 - read)
        
        P3 = kp*(error3)
        I3 =  ki*(error3)
        D3 = kd*(error3)
        control3 = P3+I3+D3
        if control3 > 1:
            control3 = 1
        elif control3 < -1:
            control3 = -1
    else:
        control3 = 0        
    
    msg = Twist()
    msg.angular.z = control1
    msg.linear.x = control3
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()