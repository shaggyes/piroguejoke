#!/usr/bin/env python
#import sys
#import time
import rospy
#import datetime
import serial
from os import popen
from math import sin, cos, pi
from subprocess import call
from std_msgs.msg import Int32MultiArray  # meuserements
from std_msgs.msg import String  # error msg
from std_msgs.msg import Float32MultiArray  # params
from nav_msgs.msg import Odometry  # odometry
from geometry_msgs.msg import Twist  # cmd_vel
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3  # odom calc

port = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=3.20,
                     write_timeout=5.5)

Timing = 10
dt = float(1)/Timing
maxIntegral = 1.0 # = limits
minPWM = 0.15  # min voltage on motors required
PIDpreverrM1 = 0  # for pid
PIDintegralM1 = 0
PIDpreverrM2 = 0
PIDintegralM2 = 0
Servo1prev = 180  # handle
Servo2prev = 20  # camera
Se1 = 180
Se2 = 20
DriveTypeFlag = 1
LedFlag = 0
recv_params =  [0, 0, 0.12, 0.168, 0.000231, 0.3, 0, 0.65, 2.2, 0.1, 1, 0, 0, 180, 0]
# Light, PWR, wheelDia, wheelBase, EncToSpeed, MaxSpeed, PorV, PID.P, PID.I,
# PID.D, DriveType, Horn, ArmPower, ServoPos1, ServoPos2
processed_speed = [.0, .0]
PoseXYth = [.0, .0, 0]

#camera = call("./SendVideo.sh", shell=True)

def sign(num):
    return -1 if num < 0 else 1
def sign0(num):
    return 0 if num==0 else (-1 if num < 0 else 1)


def callback_params(data):
    global recv_params
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    recv_params = data.data


def callback_twist(data):
    global processed_speed
    processed_speed = [data.linear.x, data.angular.z]


def filter(sensor, prev, filtConst):
    return ((sensor-prev)*filtConst + prev)


def position(dt, vx, vy, vth, x, y, th):
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt
    x += delta_x
    y += delta_y
    th += delta_th
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    #odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom_quat = [0, 0, sin(th/2), cos(th/2)]
    return Pose(Point(x, y, 0.), Quaternion(*odom_quat))


def PID(error, integral, previous_error, limits, Kp, Ki, Kd):
    global dt, maxIntegral
    integral = integral + error * dt * Ki
    if (integral > maxIntegral): # or limits
        integral = maxIntegral
    elif (integral < -maxIntegral):
        integral = -maxIntegral
    derivative = (error - previous_error) / dt
    output = float(Kp * error + integral + Kd * derivative) #*limits / 100
    previous_error = error
    if (output > limits):
        output = limits
    elif (output < - limits):
        output = -limits
    return output, previous_error, integral # error here new


def SpeedSender(speed, angle, realSpeed, realAngle):
    global recv_params, Servo1prev, Servo2prev  # drive, PorV, MaxSpeed, WheelBase, PIDs
    global PIDpreverrM1, PIDintegralM1, PIDpreverrM2, PIDintegralM2, minPWM, Se1, Se2
    global DriveTypeFlag, LedFlag
    l = ((2*speed) - (angle * recv_params[3]))/2  #command speed
    r = ((2*speed) + (angle * recv_params[3]))/2

    if (recv_params[6] == 1):  # Adaptive
        lout = sign0(l)*2.5468*abs(l)+0.128
        rout = sign0(r)*2.5468*abs(r)+0.128

    elif (recv_params[6] == 2):  # PID

        if (l > recv_params[5]):
	    l = recv_params[5]
        elif (l < -recv_params[5]):
	    l = -recv_params[5]
        if (r > recv_params[5]):
	    r = recv_params[5]
        elif (r < -recv_params[5]):
	    r = -recv_params[5]

        leftS = ((2*realSpeed) - (realAngle * recv_params[3]))/2
        rightS = ((2*realSpeed) + (realAngle * recv_params[3]))/2

        lout, PIDpreverrM1,PIDintegralM1 = PID(l-leftS, PIDintegralM1, PIDpreverrM1, 1.0,
                            recv_params[7], recv_params[8], recv_params[9])
        rout, PIDpreverrM2,PIDintegralM2 = PID(r-rightS, PIDintegralM2, PIDpreverrM2, 1.0,
                            recv_params[7], recv_params[8], recv_params[9])

    elif (recv_params[6] == 3):  # adaptive PID 20%

        if (l > recv_params[5]):
	    l = recv_params[5]
        elif (l < -recv_params[5]):
	    l = -recv_params[5]
        if (r > recv_params[5]):
	    r = recv_params[5]
        elif (r < -recv_params[5]):
	    r = -recv_params[5]

        leftS = ((2*realSpeed) - (realAngle * recv_params[3]))/2
        rightS = ((2*realSpeed) + (realAngle * recv_params[3]))/2

        lout, PIDpreverrM1,PIDintegralM1 = PID(l-leftS, PIDintegralM1, PIDpreverrM1, 1.0,
                            recv_params[7], recv_params[8], recv_params[9])
        rout, PIDpreverrM2,PIDintegralM2 = PID(r-rightS, PIDintegralM2, PIDpreverrM2, 1.0,
                            recv_params[7], recv_params[8], recv_params[9])
        lout = lout*0.2 + 0.388*(15.86**abs(l))*sign0(l)
        rout = rout*0.2 + 0.388*(15.86**abs(r))*sign0(r)

    else:
        lout = l
        rout = r

    if (lout > 1.0):
        lout = 1.0
    elif (lout < -1.0):
        lout = -1.0
    if (rout > 1.0):
        rout = 1.0
    elif (rout < -1.0):
        rout = -1.0
    if (lout < -minPWM):
        L = 60 - int(lout*51)  # magic number!! importante
    elif (lout > minPWM):
        L = int(lout*51)
    else:
        L = 0
    if (rout < -minPWM):
        R = 60 - int(rout*51)
    elif (rout > minPWM):
        R = int(rout*51)
    else:
        R = 0
    #print("\r\033[FRight: %d , Left: %d  " %(R, L))

    Servo1 = recv_params[13]
    Servo2 = recv_params[14]
    errS1 = Servo1 - Servo1prev
    errS2 = Servo2 - Servo2prev

    if (recv_params[12] != 0):
        arm = recv_params[12]
        if (arm > 1.0):
            arm = 1.0
        elif (arm < -1.0):
            arm = -1.0
        if (arm < 0):
            base = 116  # int(115.5 - 5*arm)
        else:
            base = int(110 + 5*arm)
    elif (recv_params[0] != LedFlag):
        LedFlag = recv_params[0]
        base = 122-LedFlag
    elif (recv_params[11] == 1): # Horn!!!
        base = 19
    elif (recv_params[10] == 2): # drivetype only movemtn
        base = 125
    elif (recv_params[10] != DriveTypeFlag):
        DriveTypeFlag = recv_params[10]
        base = 100 + DriveTypeFlag
    else:
        base = 110

    if (Servo1 < 200):
        if (errS1 !=0):
            Servo1prev += 5*sign0(errS1)
        Se1 = Servo1prev
    elif (Servo1 == 200):
        Se1 = 200
    elif (Servo1 == 202):
        Se1 = 202
    else: # elif (Servo1 == 280):
        Se1 = 280

    if (Servo2 < 200):
        if (errS2 !=0):
            Servo2prev += 5*sign0(errS2)
        Se2 = Servo2prev
    elif (Servo2 == 200):
        Se2 = 200
    elif (Servo2 == 202):
        Se2 = 202
    else: # elif (Servo2 == 280):
        Se2 = 280


    print("\r\033[F%d Right: %d , Left: %d  " %(base, R, L))
    return str( chr(int(base))+chr(L)+chr(R)+chr(Se1/2)+chr(Se2/2) )


def mainfunc():
    global port, processed_speed, Timing, dt, recv_params, PoseXYth
    print('step1')
  
    rospy.init_node('picore', anonymous=True)
    rate = rospy.Rate(Timing)  # 10hz
    
    pubM = rospy.Publisher('piroguemeasure', Int32MultiArray, queue_size=1)
    pubO = rospy.Publisher('odom', Odometry, queue_size=1)
    CoreMeasure = Int32MultiArray()
    CoreMeasure.data = []
    CoreOdom = Odometry()
    CoreOdom.header.frame_id = "odom"
    CoreOdom.child_frame_id = "base_link"
    
    temporarystr = port.read(7)
    prevVoltage = int(ord(temporarystr[1])*30.19+5036)
    batteryS = int(prevVoltage//3200)
    if batteryS < 1:
        batteryS = 1
    print(batteryS)
    speedRightPrev = 0
    speedRight = 0
    speedLeftPrev = 0
    speedLeft = 0
    speedBase = 0
    angleSpeed = 0
    print('step2')

    rospy.Subscriber('pirogueparams', Float32MultiArray,
                     callback_params)
    rospy.Subscriber('cmd_vel', Twist, callback_twist)  # to inwaiting

    while not rospy.is_shutdown():
        # filter here
        port.write(SpeedSender(processed_speed[0], processed_speed[1],
                               speedBase, angleSpeed))

        if port.inWaiting:

            serialstr = []
            serialstr = port.read(7)
            #print(serialstr)
            voltage = int((ord(serialstr[1]) * 30.19 + 5036))  # in mV
            prevVoltage = int(filter(voltage, prevVoltage, 0.1))
            current = int( (ord(serialstr[2]) -2) * 50)  # in mA
            VperS = float(prevVoltage)/(1000*batteryS)
            bat0l0 = int(-(188.32*(VperS**3))+(2141.2*(VperS**2)) - (7964.4*VperS)+9732.7)
            #print(bat0l0, VperS)
            tempStr = popen('vcgencmd measure_temp').readline()
            tempStr = tempStr.replace("temp=", "")
            tempStr = tempStr.replace(".", "")
            tempStr = tempStr.replace("'C\n", "")
            tempPi = int(tempStr)/10

            speedRight = ord(serialstr[4]) - ord(serialstr[3])
            speedLeft = ord(serialstr[6]) - ord(serialstr[5])

            velocityLeft = speedLeft*recv_params[4]*Timing
            velocityRight = speedRight*recv_params[4]*Timing
            speedBase = (velocityLeft + velocityRight)/2
            angleSpeed = (velocityRight - velocityLeft)/recv_params[3]

            CoreMeasure.data = [0, 0, prevVoltage,
                                bat0l0, current, tempPi, speedLeft, speedRight]
# [ But1, But2, Voltage, Charge%, Current, Temp, EncoderTicks1, Encoder2 ]
            #rospy.loginfo(CoreMeasure)
            pubM.publish(CoreMeasure)

        CoreOdom.header.stamp = rospy.Time.now()
        CoreOdom.pose.pose = position(dt, speedBase, 0, angleSpeed,
                                      PoseXYth[0], PoseXYth[1], PoseXYth[2])
        CoreOdom.twist.twist = Twist(Vector3(speedBase, 0, 0),
                                     Vector3(0, 0, angleSpeed))
        pubO.publish(CoreOdom)
        if (recv_params[1] == 1):  # shutdown now
            print("shutdown now")
            call("sudo shutdown now", shell=True)
        elif (recv_params[1] == 2): # roscore kill
            print("kill roscore")
            call("killall -9 rosmaster", shell=True)
        rate.sleep()


if __name__ == '__main__':
    try:
        mainfunc()
    except rospy.ROSInterruptException:
        pass

