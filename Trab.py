import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


kp = 0.01
ki = 0.0001
kd = 0.01
kp2 = 1
ki2 = 0.01
kd2 = 1
I1 = 0
I2 = 0
setpoint = 0
error1 = 0
old_error1 = 0
error2 = 0
old_error2 = 0
control1=0
control2=0
odom = Odometry()
scan = LaserScan()
state=0
sp=0
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
#Funcao soma de string ------------------------------------------------
def sum_digits_string(str1):
    sum_digit = 0
    for x in str1:
        if x.isdigit() == True:
            z = int(x)
            sum_digit = sum_digit + z
    return sum_digit
#--------------------------------------------------------------------	
#Somando ------------------------------------------------
Soma=0
frequencia=0
periodo=0
Soma=sum_digits_string(str('2016013120'))
Soma=sum_digits_string(str('2017001339'))+Soma
Soma=sum_digits_string(str('30550'))+Soma
Soma=sum_digits_string(str('2017020610'))+Soma
frequencia=(Soma/4)
periodo=float(1/float(frequencia))
print("Soma")
print(Soma)
print("freq")
print(frequencia)
print("periodo")
print(periodo)
#--------------------------------------------------------------------	
# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
	global kp
	global ki
	global kd
	global I1
	global I2
	global error1
	global old_error1
	global error2
	global old_error2
	global periodo
	global control1
	global control2
	global periodo
	global state
	global sp
	#Encontrando o setpoint do angulo
	if len(scan.ranges) > 0 and state == 0 and min(scan.ranges) < 3 and min(scan.ranges) > 0:
		for i in scan.ranges:	
			if i != min(scan.ranges) and state == 0:
				sp=sp+1
			else:
				state=1
		print("scanmin:")
		print(min(scan.ranges))
		print("Len:")
		print(len(scan.ranges))
		print("sp:")
		print(sp)
		print("angmin:")
		print(scan.angle_min)
		print("angmax:")
		print(scan.angle_max)
		print("Scan pos 0:")#na frente do robo
		print(scan.ranges[0])
	

	#Girando com PID 
	if state==1:
		yaw = getAngle(odom) 
		setpoint1 = sp
		print("setpoint1:")
		print(setpoint1)
		error1 = (setpoint1 - yaw)
	    
		if abs(error1) > 180:
			if setpoint < 0:
				error1 += 360 
			else:
				error1 -= 360
	        
		P1 = kp*error1
		I1 = I1 + error1 * ki
		D1 = (error1 - old_error1)*kd
		control1 = P1+I1+D1
		
		old_error1 = error1
		
		msg = Twist()
		msg.angular.z = control1
		pub.publish(msg)
		#Terminou de girar
		if len(scan.ranges) > 0 and state == 1:
			if abs(msg.angular.z)<0.0001 and msg.angular.z>0:
				state=2
				msg.angular.z=0
				print("estado")
				print(state)

	#Andando em direcao ao objeto(ate dist de 50cm) so com PID
	if state==2:
		setpoint2 = 0.5
		scan_len = len(scan.ranges)
		if state == 2:
			read = min(scan.ranges[scan_len-1 : scan_len+1])
		
			error2 = -(setpoint2 - read)
		    
			P2 = kp2*error2
			I2 = I2 + error2 * ki2
			D2 = (error2 - old_error2)*kd2
			control2 = P2+I2+D2
			if control2 > 0.1:
			    control2 = 0.1
			elif control2 < -0.1:
			    control2 = -0.1
		else:
			control2 = 0        
	    
		#Chegou em 50cm de distancia
		if len(scan.ranges) > 0 and state == 2:
			if scan.ranges[0] < 0.5:
				state=3#Acabou
				control2=0
				print("estado")
				print(state)
				
		msg = Twist()
		msg.linear.x = control2
		pub.publish(msg)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(periodo), timerCallBack)

rospy.spin()