# lego_robotics_KCLMS2017
# Code for RoboCup competition 2017 KCLMS team. 

#!/usr/bin/python3
from ev3dev import *
from ev3dev.auto import *
import time

#dribbler speed changed to 95
#code for kicker is needed
#kicker motor will be outC
#might have to change some color modes to RGB if it can't differentiate between dark green and black

'''
Notes: Use it in AC-ALL mode. When values hit > 110, discount sensor and charge forwards until no longer the case.

'''

ir = Sensor('in1:i2c8') 
gyro = GyroSensor('in2')
steer = [LargeMotor('outA'),LargeMotor('outB')]
hold = LargeMotor('outD')
ultra = UltrasonicSensor('in3')
co = ColorSensor('in4')
kmotor = LargeMotor('outC')
green=[0,0,0]




STORAGE_LENGTH = 15 # No of values in the mean
TURNINESS = 150 # Sharpness of turn
HOLD_SR_LEN = 15 # Length of shift register for holding #changed 20 to 15
CLOSENESS_THRESHOLD = 100 # in mm, allows for the sensor uncertainty 
SPINNING_SPEED = 50 # Speed that it spins at.
WHITENESS_THRESHOLD = 300


'''
Useful Functions
'''

def mean(t):
    return float(sum(t))/len(t)

def move(angle, direction=1):
    # Turn with a sharpness of TURNINESS (+TURNINESS => +angle)
    if angle > 0:
        ls = 100 - TURNINESS*angle
        rs = 100
    elif angle < 0:
        ls = 100
        rs = 100 + TURNINESS*angle 
    else:
        ls = 100
        rs = 100
    steer[0].run_direct(duty_cycle_sp=int(ls)*direction)
    steer[1].run_direct(duty_cycle_sp=int(rs)*direction)

def spin(direction):
    steer[0].run_direct(duty_cycle_sp=SPINNING_SPEED*direction)
    steer[1].run_direct(duty_cycle_sp=-SPINNING_SPEED*direction)

def stop():
    steer[0].stop()
    steer[1].stop()

def getGyro():
    # gyroValue takes values of -180 to +180 inclusive
    return ((gyro.value() + 180) % 360) - 180

def resetGyro():
    gyro.mode = "GYRO-RATE"
    gyro.mode = "GYRO-ANG"

def getIR():
    v = ir.value(0)
    if v == 7:
        return 6
    else:
        return v

def isWhite():
    co.mode = "RGB-RAW"
    return sum([co.value(i) for i in range(3)]) > WHITENESS_THRESHOLD

def isGreen():
    co.mode = "RGB-RAW"
    isGreen=True
    for i in range(0,3):
        if abs(co.value(i) - green(i)) > 20: #threshold for within green is 20 test this and change
            isGreen=False
    return isGreen

def isColor(color):
    co.mode = "COL-COLOR"
    colorval = {"No color": 0, "Black": 1, "Blue": 2, "Green": 3, "Yellow": 4, "Red":5, "White":6,"Brown": 7}
    return co.value(0)==colorval[color]
    

def hasBall(ro):
    ro.holdValues[ro.holdPointer] = hold.speed
    ro.holdPointer += 1
    if ro.holdPointer >= HOLD_SR_LEN:
        ro.holdPointer = 0
    if mean(ro.holdValues) <= ro.holdThreshold:
        return ro, True
    else:
        return ro, False

def collision(ro):
    pass

'''
States
'''



def moveToGoal(ro):
    while True:
        move(0)
        ro, gotBall = hasBall(ro)
        if not gotBall:
            return "lookForBall", ro
        if abs(getGyro()) > 45:
            return "realign", ro 
        ul = ultra.value()
        if ul < CLOSENESS_THRESHOLD:
            return "retreat", ro
        return "goalsearch", ro

def goalsearch(ro):#function to search for the goal when facing forward but can't see goal
    #STAGE 1 said straighten the robot
    x=realign(ro) #x is just holding values so realign runs, don't care about x
    stop()
    #turns to the right and then approaches right hand side(side line)
    print("turning to right side line") 
    time.sleep(1)
    while getGyro() < 90:
        spin(1)
    
    print("moving to side line")
    while not isWhite():
        gyroValue = getGyro()
        ro, gotBall = hasBall(ro)
        if not gotBall:
            return "lookForBall", ro
        if gyroValue < 100 and gyroValue>80: #allowing for uncertainty in gyro sensor WARNING: significant uncertainty in hardware, need to keep that in mind
            move(0)
        else:
            if gyroValue<80:
                while getGyro() < 90:
                    spin(-1)
            if gyroValue>100:
                while getGyro() > 90:
                    spin(1) 
        ul=ultra.value()
        if ul<CLOSENESS_THRESHOLD:  # if when moving to the sideline it gets close to something...
            print("theres a robot Run away")
            return "retreat", ro #it must be a robot so retreat
        
        #found white side line,
    print("Goal search stage 2, approaching front line")
    x,y=realign(ro)
    while not isWhite():
        move(0)
        ul=ultra.value()
        if ul<CLOSENESS_THRESHOLD:  # if when moving to the sideline it gets close to something...
            print("theres a robot RUN AWAY")                
            return "retreat", ro #it must be a robot so retreat
        if abs(getGyro()) > 45:
            x,y = realign(ro)
    

    print("looking for penalty box")
    move(0,-1)
    time.sleep(0.5)
    stop()
    while getGyro() >-90:
        spin(-1)
     
    while isGreen()== True:
        move(0)
     #found penalty box
    print("in penalty box")
    #facing the left, approaches center of pitch

    move(0)
    time.sleep(3)#need to check if we can get to the center in that time
    stop()#now in the center 

    #goes to moveToGoal mode so it checks if a goal is there
    while True:
        if abs(getGyro()) > 45:
            x,y=realign(ro)
        move(0)
        ro, gotBall = hasBall(ro)
        if not gotBall:
            return "lookForBall", ro
        return "shoot", ro




def shoot(ro):
    # Fire all motors forwards! On losing the ball, swap to findBall.
    move(0)
    hold.run_direct(duty_cycle_sp=-1) 
    setkicker(ro)
    time.sleep(0.5)
    return "lookForBall", ro

def lookForBall(ro):
    # If ball's in sight, go get it, else spin. On finding it, realign.
    global SENSOR_MODE
    while True:
       # SENSOR_MODE = setcolorsensor(SENSOR_MODE, "down")
        ro, gotBall = hasBall(ro)
        irValue = getIR()
        if irValue != 0 and not isWhite():
            ro.irValues[ro.irPointer] = irValue
            ro.irPointer += 1
            if ro.irPointer >= STORAGE_LENGTH:
                ro.irPointer = 0
            angle = (irValue-5)*0.25
            move(angle)
        elif isWhite():
            print("Saw line.")
            # Back away a bit, then spin.
            move(0,-1)
            time.sleep(0.75)
            spin(1)
            time.sleep(0.75)
        else:
            if mean(ro.irValues) > 5:
                move(1)
            else:
                move(-1)
        hold.run_direct(duty_cycle_sp=95) #####
        # Check if it's found:
        if gotBall:
            return "realign", ro
        ul=ultra.value()
        if ul<CLOSENESS_THRESHOLD:  # if when moving to the sideline it gets close to something...
            print("theres a robot Run away")
            return "retreat", ro #it must be a robot so retreat

def retreat(ro):

    # We hit "the wall", reverse slightly, turn 90d, move forwards a bit (this is blocking). Then go to realign.

    move(0,-1)

    time.sleep(0.5)

    #while abs(getGyro()) < 90:
    #             spin(1)
      
    if getGyro() > 30:
        while getGyro() > 30:
            move(0,-1)
            time.sleep(0.5)
            spin(-1)
    elif getGyro()  < -30:
        while getGyro() < -30:
            move(0,-1)
            time.sleep(0.5)
            spin(1)

                

    move(1)
    

    for i in range(20):
        time.sleep(0.075)
        if isWhite():
            return "retreat", ro

    return "realign", ro


#maybe less than 90 degrees rotation
#maybe increase the speed

def setkicker(ro):
    kmotor.run_timed(time_sp=125, speed_sp=-360)
    time.sleep(0.5)
    kmotor.run_timed(time_sp=125, speed_sp=360)
        



def realign(ro):
    # If angle is far off, rotate until not the case.
    while True:
        gyroValue = getGyro()
        ro, gotBall = hasBall(ro)
        if not gotBall:
            return "lookForBall", ro
        if abs(gyroValue) < 5:
            return "moveToGoal", ro
        else:
            spin(-gyroValue/abs(gyroValue))

def reset(ro):
    # Reset the robot as it's been picked up.
    resetGyro()
    return "lookForBall", ro

class RobotObject():
    def __init__(self):
        self.irValues = [5] * STORAGE_LENGTH
        self.irPointer = 0
        self.holdThreshold = 0
        self.holdValues = [1000] * HOLD_SR_LEN
        self.holdPointer = 0

if __name__ == '__main__':
    functions = {"moveToGoal": moveToGoal, "retreat": retreat, "shoot": shoot, "lookForBall": lookForBall, "realign": realign, "goalsearch":goalsearch}
    ro = RobotObject()
    ir.mode = "AC-ALL"
    co.mode = "RGB-RAW"
    state = "lookForBall"
    print("GO!")
    holdingSr=[] # This is our general shift register, I'll explain later.
    assert steer[0].connected and steer[1].connected and hold.connected # Basic assert, just in case
    hold.run_direct(duty_cycle_sp=95) # This is our dribbler motor
    hThreshold = 0
    resetGyro()
    time.sleep(1)
    for _ in range(HOLD_SR_LEN): # Check the dribbler speed to work out when the ball is in the way WARNING: should potentially change the thresholds if the motors are not sensitive enough 
        holdingSr.append(hold.speed)
        
    green[1]=co.value(1) #dealing with colour. Might need to change to RGB values if the pitch is a different colour to what we are used to 
    green[2]=co.value(2)
    green[0]=co.value(0)
        
    for i in range(10):
        green[1]=(green[1] + co.value(1)) / 2
        green[2]=(green[2] + co.value(2)) / 2
        green[0]=(green[0] + co.value(0)) / 2
                  
    hThreshold = sum(holdingSr)/(HOLD_SR_LEN + 1.0)
    ro.holdThreshold = hThreshold
    while True:
        print(state)
        state,ro = functions[state](ro)

