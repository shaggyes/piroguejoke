#!/usr/bin/env python
import pygame
import datetime
import sys
import rospy
import tf
import math
from math import sin, cos, pi, acos, asin
from std_msgs.msg import Int32MultiArray    # meuserements
from std_msgs.msg import Float32MultiArray  # params
from std_msgs.msg import String    # error msg
from nav_msgs.msg import Odometry   # odometry
from geometry_msgs.msg import Twist  # cmd_vel
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import BatteryState    # battery

pygame.init()
# preprocedure
# pygame settings
pygame.display.set_caption('Keyboard for robot pobot')
size = [640, 480]
screen = pygame.display.set_mode(size)
clock = pygame.time.Clock()
# by default the key repeat is disabled call set_repeat() to enable it
pygame.key.set_repeat(50, 50)
robot_skin = pygame.image.load('catkin_ws/src/piroguejoke/src' +
                               '/robotskin.png').convert()
robot_skin.set_colorkey((0, 0, 0))
robot_skin = pygame.transform.scale(robot_skin,
                                    (robot_skin.get_width()//2,
                                     robot_skin.get_height()//2))
# graphics
# colors
white = (255, 255, 255)
yellow = (245, 240, 90)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (10, 10, 250)
darkviolet = (120, 3, 120)
violet = (200, 10, 160)
grey = (160, 160, 160)
darkgrey = (120, 120, 120)
black = (5, 0, 20)
lightred = (255, 200, 200)
lightgreen = (200, 255, 200)
darkred = (155, 0, 0)
orangedark = (200, 150, 0)
darkgreen = (0, 155, 0)
darkblue = (0, 0, 155)
bluishred = (235, 100, 30)
contrastgreen = (20, 175, 20)
popblue = (55, 0, 200)
bluishblue = (30, 100, 230)
# rendered text and pic
myfont = pygame.font.SysFont("monospace", 15)
fontmid = pygame.font.SysFont("Arial", 30)
GOfont = pygame.font.SysFont("monospace", 90)
GOlabel = GOfont.render(str("GAME OVER"), 1, black)
text_filtered = myfont.render("Filtered", 1, violet)
text = ["key LEFT, RIGHT, UP, DOWN - control movement of robot",
        "L_Shift - decrease speed limit", "R_Shift increase speed limit",
        "TAB - increase speed limit to 100% - 'turbo mode'",
        "F - turn on/off low-pass filter for robots params",
        "O - turn on/off odometry in meters", "P - PID turn on/off",
        "<,> - Arm movement", "SPACE - Servo on/off",
        "W,S - Servo1 movement | A,D - Servo2 movement",
        "Q - Servo1 on/off | E - Servo1 on/off",
        "1, 3 - first person view/ third",
        "V - light on/off", "H - horn",
        "Backspace - exit of this application",
        "Delete - exit of this application",
        "ESC - Raspberry Pi shutdown signal",
        " ", " ", "  > Press any button to close legend <"]
legendary = []
for line in text:
	legendary.append(myfont.render(line, True, white))
warningtext = myfont.render("WARNING! BATT disCHG", 1, red)
textASA = myfont.render("Servo available", 2, red)
textASO = myfont.render("All Servo off", 2, red)
textRosKill = myfont.render("Roscore killed", 2, red)
textPressL = myfont.render("Press 'L' for legend", 2, grey)

# params defined
params_common = [0, 0, 0.12, 0.165, 0.000231, 0.3, 0, 0.65, 2.2, 0.1, 1, 0, 0, 180, 30]
# Light, PWR, wheelDia, wheelBase, EncToSpeed, MaxSpeed, PorV, PID.P, PID.I,
# PID.D, DriveType, Horn, ArmPower, ServoPos1, ServoPos2

# var list
inverseKey = -1
Horn = 0
armSpeed = 0
Light = False
Servo1q = 0
Servo2e = 0
Servo1 = 180.0
Servo2 = 20.0
xServo2 = 0
yServo2 = 0
minServo = 0
maxServo = 180
trigger_animation = True
fps = 10    # FPS!!!!
timePi = 0
timeComp = 0
M1 = 0      # speed for left board
M2 = 0
k = 0.5     # from max speed
prevvoltage = 0     # filter vars
batteryS = 3
prevcurr = 0
odomSpeed = 0
odomAngle = 0
odomDistance = 0
odomX = 0
odomY = 0
odomTheta = 0
deltatoPi = 0
deltafromPi = 0
odometerBtn = False     # btns on comps keyboard
btnFilterOn = False
legend = False
PIDbtn = 0 # 0 - pwm, 1 - adaptive, 2 - pid, 3 - adaptive pid
OnOff = 0
FullDrivebtn = 1
AnimationCounter = 0    # temporary
ArmAnimation = 30
dt = float(1)/fps   # integral part
# callback vars
measuresCore = [0, 0, 8000, 50, 100, 19, 0, 0]
# [ But1, But2, Voltage, Charge%, Current, Temp, EncoderTicks1, Encoder2 ]


# legend of keyboard
def legend_show():
    screen.fill(black)
    for line in range(len(legendary)):
        screen.blit(legendary[line], (100, 100+line*15))


# low pass filter common
def filter(sensor, prev, filtConst):
    return ((sensor-prev)*filtConst + prev)


def callback_measures(data):
    global measuresCore
    measuresCore = data.data


# odomList = [0, 0, 0, 0, 0] #speedx, anglespeed, posX, posY, w
def callback_odometry(data):
    global odomX, odomY, odomTheta, odomSpeed, odomAngle, timePi
    timePi = data.header.stamp
    odomSpeed = data.twist.twist.linear.x
    odomAngle = data.twist.twist.angular.z
    odomX = data.pose.pose.position.x
    odomY = data.pose.pose.position.y
    odomTheta = acos(data.pose.pose.orientation.w)*2


# function that changes vars by butns are pressed
def keyboardCheck():
    global k, M1, M2, btnFilterOn, odometerBtn, legend, PIDbtn, \
           OnOff, Horn, FullDrivebtn, AnimationCounter, ShowImubtn, \
           Servo1, Servo2, Servo1q, Servo2e, Light, armSpeed, ArmAnimation
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                M1 = k
                M2 = -k
            elif event.key == pygame.K_RIGHT:
                M1 = -k
                M2 = k
            if event.key == pygame.K_UP:
                M1 = k
                M2 = k
            elif event.key == pygame.K_DOWN:
                M1 = -k
                M2 = -k
            #else:
            #    M1 = 0
            #    M2 = 0
                #AnimationCounter = 0
            if event.key == pygame.K_COMMA:
                armSpeed = -1
            elif event.key == pygame.K_PERIOD:
                armSpeed = 1
            else:
                armSpeed = 0
            if event.key == pygame.K_w:
                Servo1 += 5
            elif event.key == pygame.K_s:
                Servo1 -= 5
            if event.key == pygame.K_a:
                Servo2 += 5
            elif event.key == pygame.K_d:
                Servo2 -= 5
            if (Servo2 > maxServo):
                Servo2 = maxServo
            elif (Servo2 < minServo):
                Servo2 = minServo
            if (Servo1 > maxServo):
                Servo1 = maxServo
            elif (Servo1 < minServo):
                Servo1 = minServo

            if pygame.key.get_mods() & pygame.KMOD_LSHIFT:
                k = k - 0.05
            elif event.key == pygame.K_TAB:
                k = 1
            elif pygame.key.get_mods() & pygame.KMOD_RSHIFT:
                k = k + 0.05
            elif event.key == pygame.K_ESCAPE:
                OnOff = 1
            elif event.key == pygame.K_DELETE:
                OnOff = 2
            elif event.key == pygame.K_f:
                if (btnFilterOn is False):
                    btnFilterOn = True
                else:
                    btnFilterOn = False
            elif event.key == pygame.K_o:
                if (odometerBtn is False):
                    odometerBtn = True
                else:
                    odometerBtn = False
            elif event.key == pygame.K_p: # drive type change
                if (PIDbtn == 0):
                    PIDbtn = 1
                else:
                    PIDbtn = 0
            elif event.key == pygame.K_9:
                PIDbtn = 2
            elif event.key == pygame.K_0:
                PIDbtn = 3
            elif event.key == pygame.K_v:
                if (Light is False):
                    Light = True
                else:
                    Light = False
            elif event.key == pygame.K_SPACE:
                if (FullDrivebtn == 0):
                    FullDrivebtn = 1
                else:
                    FullDrivebtn = 0
            elif (event.key == pygame.K_l):
                legend = True
            elif event.key == pygame.K_h:
                Horn = 1
            elif event.key == pygame.K_q:
                if (Servo1q == 202):
                    Servo1q = 200
                elif (Servo1q == 200):
                    Servo1q = 0
                else:
                    Servo1q = 202
            elif event.key == pygame.K_e:
                if (Servo2e == 202):
                    Servo2e = 200
                elif (Servo2e == 200):
                    Servo2e = 0
                else:
                    Servo2e = 202
            elif event.key == pygame.K_1:
                Servo2 = 20
                Servo1 = 180
            elif event.key == pygame.K_3:
                Servo2 = 160
                Servo1 = 30
            else:
                legend = False
            if event.key == pygame.K_BACKSPACE:
                pygame.quit()
                sys.exit()
            AnimationCounter += 1
        else:
            M1 = 0
            M2 = 0
            armSpeed = 0
            AnimationCounter = 0
            Horn = 0
        if k > 1:
            k = 1
        elif k < 0.1:
            k = 0.1
        

# convert diff drive to steering ackerman
def representSpeed(Motor1, Motor2, BaseWidth, MaxSpeed=params_common[5]): 
    global PIDbtn
    X = (Motor1 + Motor2)/2
    W = (Motor2 - Motor1)/BaseWidth
    if (PIDbtn != 0):
        X = X*MaxSpeed
    return X, W


# func shows media
def blitting():
    global ArmAnimation
    # camera hand
    if (Servo1 == 202):
        Srv1adptd = 0
    else:
        Srv1adptd = 185-Servo1
    Srv2adptd = 100-Servo2
    xServo2 = 150*cos(Srv1adptd/57.3)-10*sin(Srv1adptd/57.3)
    yServo2 = 10*cos(Srv1adptd/57.3)+150*sin(Srv1adptd/57.3)
    pygame.draw.polygon(screen,grey,[(420, 200), (420-10*sin(Srv1adptd/57.3),
                        200-10*cos(Srv1adptd/57.3)), 
                        (420+xServo2, 200-yServo2), (420+150*cos(Srv1adptd/57.3), 
                        200-150*sin(Srv1adptd/57.3))],False)
    pygame.draw.circle(screen, darkgrey, (420-int(5*sin(Srv1adptd/57.3)), 
                       200-int(5*cos(Srv1adptd/57.3))), 10)
    pygame.draw.line(screen, green, (420+xServo2, 200-yServo2), 
                     (420+xServo2-40*sin((Srv2adptd+Srv1adptd-90)/57.3), 
                      200-yServo2-40*cos((Srv2adptd+Srv1adptd-90)/57.3)), 8)
    pygame.draw.circle(screen, darkgrey, (420+int(xServo2), 200-int(yServo2)), 9)                    
    pygame.draw.line(screen, darkgrey, (420-int(5*sin(Srv1adptd/57.3)), 
                       200-int(5*cos(Srv1adptd/57.3))), (420+int(15*sin(Srv1adptd/57.3)), 
                       200+int(15*cos(Srv1adptd/57.3))), 8)
    screen.blit(myfont.render("S1 "+str(int(Servo1))+"'", 1, blue), (390-int(5*sin(Srv1adptd/57.3)), 225))
    screen.blit(myfont.render("S2 "+str(int(Servo2))+"'", 1, darkred), (540, 225))
    if ((Servo1 > 170) and (Servo1 < 190)):
        screen.blit(myfont.render("Q", 1, black), (510, 156))
    # arm indication
    if (armSpeed < 0):
        pygame.draw.rect(screen, lightred, ((30, 160), (200, 100)), 0)
        ArmAnimation -=5
        pygame.draw.line(screen, black, (145, 210), (145, 191), 3)
    elif (armSpeed > 0):
        pygame.draw.rect(screen, lightgreen, ((30, 160), (200, 100)), 0)
        ArmAnimation +=5
        pygame.draw.line(screen, black, (145, 210), (145, 191), 3)
    else:
        ArmAnimation = 30
    screen.blit(myfont.render("Arm speed "+str(armSpeed*100), 8, black), (70, 280))

    pygame.draw.line(screen, (65, 61, 65), (81, 204), (158, 204), 26)
    #pygame.draw.line(screen, black, (85, 210), (124, 210), 3)
    #pygame.draw.line(screen, black, (65, 191), (145, 191), 3)
    pygame.draw.circle(screen, black, (85, 210), 22, 0)
    pygame.draw.circle(screen, green, (155, 210), 22, 0)
    pygame.draw.circle(screen, green, (85, 210), 10, 0)
    pygame.draw.circle(screen, black, (155, 210), 12, 0)
    pygame.draw.line(screen, darkgrey, (155, 210), (155+43*cos(ArmAnimation/57.3), 
                     210+43*sin(ArmAnimation/57.3)), 8)   
    # Light
    if (Light is True):
        pygame.draw.circle(screen, yellow, (145, 100), 20, False) 
        screen.blit(fontmid.render("L", 8, darkviolet), (137, 85))
    # buttons f and space
    if (btnFilterOn is True):
        screen.blit(text_filtered, (400, 410))
    if (FullDrivebtn == 0):
        screen.blit(textASO, (500, 410))
    elif ((Servo1q > 0) or (Servo2e > 0)):
        if (Servo1q == 200):
            screen.blit(myfont.render("S1 att", 2, blue), (500, 410))
        elif (Servo1q == 202):
            screen.blit(myfont.render("S1 off", 2, red), (500, 410))
        else:
            screen.blit(myfont.render("S1 cnt", 2, darkgreen), (500, 410))
        if (Servo2e == 200):
            screen.blit(myfont.render("S2 att", 2, blue), (560, 410))
        elif (Servo2e == 202):
            screen.blit(myfont.render("S2 off", 2, red), (560, 410))
        else:
            screen.blit(myfont.render("S2 cnt", 2, darkgreen), (560, 410))   
    else:
        screen.blit(textASA, (490, 410))
    if (OnOff == 2):
        screen.blit(textRosKill, (15, 15))

    # params:
    text_volt = myfont.render(str(float('{:.2f}'.format(prevvoltage/1000))) +
                              "V", 1, violet)
    text_curr = myfont.render(str(float('{:.2f}'.format(prevcurr/1000))) + "A",
                              1, (15, 200, 150))
    text_timeComp = myfont.render("ping:"+str(deltafromPi)+"ms", 1, (115, 150, 0))
    text_tempPi = myfont.render("tPi:" + str(float(measuresCore[5]))+"c", 1, bluishblue)
    
    if (measuresCore[3] > 10):
        text_bat0l0 = myfont.render("BATT." + str(batteryS) + "sLiPo " +
                                    str(measuresCore[3])+"%", 1, orangedark)
        screen.blit(text_bat0l0, (50, 410))
    else:
        screen.blit(warningtext, (50, 410))
    screen.blit(text_volt, (50, 320))
    screen.blit(text_curr, (140, 320))
    screen.blit(text_tempPi, (50, 345))
    screen.blit(text_timeComp, (50, 370))
    # odometry:
    if (PIDbtn == 0):
        text_limitSpeed = myfont.render("Speed " + str(k*100)+"% PWM",
                                        1, contrastgreen)
    else:
        text_limitSpeed = myfont.render("Speed " + str(k*(params_common[5])) +
                                        "m/s", 1, contrastgreen)
    screen.blit(text_limitSpeed, (440, 300))
    text_speeed = myfont.render(str('{:.2f}'.format(odomSpeed))+" m/s",
                                1, contrastgreen)
    text_speeedw = myfont.render(str('{:.1f}'.format(odomAngle))+" rad/s",
                                 1, contrastgreen)
    screen.blit(text_speeed, (440, 330))
    screen.blit(text_speeedw, (440, 350))
    if (odometerBtn is True):
        screen.blit(myfont.render("Distance: " +
                                  str('{:.2f}'.format(odomDistance)) +
                                  " m", 1, darkred), (440, 370))
        screen.blit(myfont.render("X:" + str('{:.1f}'.format(odomX)) +
                                  " | Y:" + str('{:.1f}'.format(odomY)),
                                  1, orangedark), (440, 390))
    # draw a robots background
    if (M1 < 0):
        pygame.draw.rect(screen, lightred, ((320, 290), (60, 135)), 0)
    elif (M1 > 0):
        pygame.draw.rect(screen, lightgreen, ((320, 290), (60, 135)), 0)
    if (M2 < 0):
        pygame.draw.rect(screen, lightred, ((260, 290), (60, 135)), 0)
    elif (M2 > 0):
        pygame.draw.rect(screen, lightgreen, ((260, 290), (60, 135)), 0)
    # draw a robot rotation

    if (trigger_animation is True):
	    if (M1 == 0 and M2 == 0):
		screen.blit(robot_skin, (275, 310))
	    else:
		rotSkinRobot = pygame.transform.rotate(robot_skin,
		                                       AnimationCounter*(M1-M2))
		screen.blit(rotSkinRobot, (320-(rotSkinRobot.get_width()//2),
		                           360-(rotSkinRobot.get_width()//2) -
		                           ((M1+M2)/2*AnimationCounter) % 10))
    else:
	    rotSkinRobot = pygame.transform.rotate(robot_skin, odomTheta)
	    screen.blit(rotSkinRobot,
		        (320 - (rotSkinRobot.get_width()//2) -
		         (sin(odomTheta)*odomSpeed*AnimationCounter % 10)),
		        (360 - (rotSkinRobot.get_width()//2) -
		         (cos(odomTheta)*odomSpeed*AnimationCounter % 10)))

    # game over
    if (OnOff == 1):
        screen.blit(GOlabel, (50, 180))
    if (legend is True):
        legend_show()
    screen.blit(textPressL, (450, 2))

# func for evaluations
def calculatingParams():
    global prevcurr, prevvoltage, odometerBtn, odomDistance, fps, rangesScan,\
           filtrSCANdata, timeComp, deltafromPi # , deltatoPi
    #print(rangesScan)
    # odometry
    if (odometerBtn is False):
        odomDistance = 0
    else:
        odomDistance += abs(odomSpeed/fps)
    # low pass filter
    if (btnFilterOn is False):
        prevcurr = filter(measuresCore[4], prevcurr, 0.1)
        prevvoltage = filter(measuresCore[2], prevvoltage, 0.5)
    else:
        prevcurr = filter(measuresCore[4], prevcurr, 0.05)
        prevvoltage = filter(measuresCore[2], prevvoltage, 0.1)
    # delay from pc&pi
    timeComp = rospy.Time.now()
    deltafromPi = ((timeComp - timePi).to_nsec())//1000000  # in ms
    # measuresCore[last] # corecontrol.timenow - twiststamped


# ############################################## main part here #####
def mainfunc():
    global params_common, timeComp, M1, M2, OnOff, PIDbtn,\
           FullDrivebtn, timePi
    print('step0')
    rospy.init_node('pccontrollite', anonymous=True)
    timePi = rospy.Time.now()
    #pubC = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    pubC = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pubP = rospy.Publisher('pirogueparams', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(fps)  # 20hz
    #PultSpeed = Twist()
    PultSpeed = Twist()
    PultParams = Float32MultiArray()

    rospy.Subscriber('piroguemeasure', Int32MultiArray, callback_measures)
    rospy.Subscriber('odom', Odometry, callback_odometry)

    while not rospy.is_shutdown():
        # batteryS = int(prevvoltage//3200)

        keyboardCheck()
        calculatingParams()
        screen.fill(white)
        blitting()
        pygame.display.update()
        params_common[0] = Light
        params_common[1] = OnOff
        params_common[6] = PIDbtn
        params_common[10] = FullDrivebtn
        params_common[11] = Horn
        if ((Servo2e == 200) or (Servo2e == 202)):
            params_common[14] = Servo2e
        else:
            params_common[14] = Servo2
        if ((Servo1q == 200) or (Servo1q == 202)):
            params_common[13] = Servo1q
        else:
            params_common[13] = Servo1
        params_common[12] = armSpeed
        PultParams.data = params_common
        pubP.publish(PultParams)
        PultSpeed.linear.x, PultSpeed.angular.z = \
            representSpeed(M1, M2, params_common[3])
        pubC.publish(PultSpeed)
        rate.sleep()
        #print(rospy.Time.now())

if __name__ == '__main__':
    try:
        mainfunc()
    except rospy.ROSInterruptException:
        pass

