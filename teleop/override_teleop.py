# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       whyi8                                                        #
# 	Created:      7/29/2026, 6:17:51 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

"""teleop but we have a cascade lift"""


# Library imports
from vex import *

brain = Brain()
controller_1 = Controller(PRIMARY)

# ports
"""LEFT and RIGHT are determined by looking at the robot in the perpsective where the claw is facing you"""

# drivetrain
motor_FR = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
motor_MR = Motor(Ports.PORT15, GearSetting.RATIO_6_1, True); "'middle' ones are the 5.5W motors"
motor_BR = Motor(Ports.PORT16, GearSetting.RATIO_6_1, True)
motor_FL = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
motor_ML = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
motor_BL = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False)

# cascade
lift_R = Motor(Ports.PORT12, GearSetting.RATIO_36_1, True)
lift_L = Motor(Ports.PORT20, GearSetting.RATIO_36_1, False)

# intake
intakeMotor = Motor(Ports.PORT9, GearSetting.RATIO_36_1, True)

# claw
clawP = DigitalOut(brain.three_wire_port.h); "open by default"

dx = 20

# lets set these velocities wooooo
# lift shenanigans
lift_L.set_position(0, DEGREES)
lift_R.set_position(0, DEGREES)
lift_L.set_max_torque(50, PERCENT)
lift_R.set_max_torque(50, PERCENT)
#liftV = 100
#lift_L.set_velocity(liftV)
#lift_R.set_velocity(liftV)

intendedDegree = 0

intakeV = 100
intakeMotor.set_velocity(intakeV)


startingTurnVelocity = 28 #default is 25
otherTurnVelocity = 35

a = 18 #quadratic
b = 80 #linear
c = 2 #verticalTranslation
d = 1 #deadzone
p = 5 #power


def setSpeed(leftV,rightV):

        #sets the velocities of all the motors to the correct amount
        motor_FL.set_velocity(leftV, PERCENT);
        motor_FR.set_velocity(rightV, PERCENT);
        motor_BL.set_velocity(leftV, PERCENT);
        motor_BR.set_velocity(rightV, PERCENT);
        motor_ML.set_velocity(leftV, PERCENT);
        motor_MR.set_velocity(rightV, PERCENT);

# thread tiiimmee
# controls the chassis drivetrain whatever

#Basic input curve without any translations
def inputCurveRaw(input, a, b, p):
    y = (a/100)*pow(input,p) +  (b/100)*input
    return y


#Modified input curve using deadzones and the velocity to initially move the motor
def inputCurve(input, a, b, c, d, p):

    if(input >= d/100):
        #Modified input
        y = inputCurveRaw((1+d/100)*(input-d/100), a,b,p) + c/100

    elif (input <= -d/100):
        input *= -1
        y = -inputCurveRaw((1+d/100)*(input-d/100), a,b,p) - c/100

    else:
        y = 0

    return y

def drive():     #Threaded function to drive motors based on controller input

    #Defines how fast robot should turn in %
    turnVelocity = startingTurnVelocity 

    toggle = False
    toggle2 = False
    

    slow = False


    maxVelocity = 100


    #changes how much of the turning velocity is kept at high speeds
    turnSpeedMult = 1.1

    #While loop because the function is threaded
    while (True):
        #print("\033[2J") #Clears console
        #tipPrevention()
        
        #Macros
        


        #if statement to allow changes to turn velocity while driving robot

        """if (controller_1.buttonX.pressing() and toggle2 == False):
            if (slow == True):
                slow = False
                turnVelocity = startingTurnVelocity

                #Telemetry for drivers
                controller_1.screen.clear_row(1)
                controller_1.screen.set_cursor(1, 1)
                controller_1.screen.print("turn velocity " + str(startingTurnVelocity)) 
                print("turn velocity " + str(startingTurnVelocity))


            elif (slow == False):
                slow = True
                    
                turnVelocity = otherTurnVelocity

                #Telemetry for drivers
                controller_1.screen.clear_row(1)
                controller_1.screen.set_cursor(1, 1)
                controller_1.screen.print("turn velocity " + str(otherTurnVelocity))
                print("turn velocity "+ str(otherTurnVelocity))
                

            toggle2 = True

        elif ( controller_1.buttonX.pressing() == False and toggle2 == True):
            toggle2 = False"""


        #THIS TOGGLES THE CALIBRATION TO CHECK IF TOGGLED READ THE BOOLEAN VALUE OF "cal_on"
        '''
        if ( controller_1.buttonY.pressing() and toggle == False):
            
            if (cal_on == True):
                cal_on = False

            elif (cal_on == False):
                cal_on = True
            
            toggle = True

        elif ( controller_1.buttonY.pressing() == False and toggle == True):
            toggle = False
        '''

       


        #calculates left and right drivetrain velocity based on inputs and turn velocity

        forwardV = inputCurve(controller_1.axis3.position()/100,a,b,c,d,p)
        leftV = -100*forwardV + turnVelocity * -controller_1.axis1.position()/100
        rightV = -100*forwardV - turnVelocity * -controller_1.axis1.position() /100

        #note: axis3 is the forward axis and axis1 is the turning axis.


        #Checks if drivetrain velocity for either side is bigger than 100. If it is it adds it subtracts it from the other side to keep the robot turning the same speed.
        #better explained on july 8 in the engineering notebook

        if (leftV > maxVelocity):
            rightV -= turnSpeedMult*(leftV- maxVelocity)
            leftV = maxVelocity

        elif (leftV < -maxVelocity):
            rightV -= turnSpeedMult*(leftV + maxVelocity)
            leftV = -maxVelocity
                

        if (rightV > maxVelocity):
            leftV -= turnSpeedMult*(rightV- maxVelocity)
            rightV = maxVelocity

        elif (rightV < -maxVelocity):
            leftV -= turnSpeedMult*(rightV + maxVelocity)
            rightV = -maxVelocity


        #for debugging        
        #print("|" + str(round(leftV)) + "|-|" + str(round(rightV)) + "|   " + str(controller_1.axis1.position()) + "|" + str(controller_1.axis3.position()) + "   "  +  "   Turn velocity:" + str(turnVelocity))

        #sets speed for the drivetrain
        setSpeed(leftV,rightV)

        #prevents the loop from taking up all the brains resources.
        wait(15, MSEC)

"""logarithmic speed?"""
# controls the double reverse 4 bar lift
def d4rb():


    intendedDegree = 0
    lift_L.set_velocity(100)
    lift_R.set_velocity(100)

    """def needs to be changed"""
    maxLiftDegree = 175 #basically what degree the motors would be if the d4rb was extended to full height (~4.5 cup/pin stack)

    while True:
        averageMotorDegree = (lift_L.position() + lift_R.position())/2 #formula might need to be changed on ts because one of the motors is probably reversed"

        # 100% is 127 rpm or 762 dps; 50% is 63.5 rpm or 381 dps
        liftV = 50


        # go UP
        if (controller_1.buttonL1.pressing()):

            #intendedDegree += (liftV/100*762/(1000/dx))*2 # convert velocity to degrees/ms

            """ if (intendedDegree > maxLiftDegree):
                intendedDegree = 170"""

            
            """print(intendedDegree)
            lift_L.spin_to_position(intendedDegree, DEGREES)
            lift_R.spin_to_position(intendedDegree, DEGREES)"""

            lift_L.spin(FORWARD)
            lift_R.spin(FORWARD)

        #go DOWN
        elif (controller_1.buttonL2.pressing()):
            


            #intendedDegree -= (liftV/100*762/(1000/dx))*2 # convert velocity to degrees/ms

            """if (intendedDegree < 0):
                intendedDegree = 0"""

            intendedDegree = 0

            #print(intendedDegree)
            lift_L.spin(REVERSE)
            lift_R.spin(REVERSE)

            #lift_L.spin(REVERSE)
            #lift_R.spin(REVERSE)
        else:
            lift_L.stop()
            lift_R.stop()

        # PID loop to keep position in place?

        print("lift_L temp:", str(lift_L.temperature()) + "; lift_R temp:", lift_R.temperature())
        wait(15, MSEC)

"""buttons to change speed??"""
"""is there ever gonna be a situation where the intake needs to spin in reverse.....unless it gets stuck"""    
# controls the intake
def intake():

    while True:
        if (controller_1.buttonA.pressing):
            intakeMotor.spin(FORWARD)
        wait(20, MSEC)

# controls the claw

def claw():

    clamping = False
    delay = False
    

    while True:
        if (controller_1.buttonR1.pressing() and not delay):

            if (not clamping):
                clawP.set(True)
                clamping = True

            elif (clamping):
                clawP.set(False)
                clamping = False

            delay = True
        elif (not controller_1.buttonR1.pressing() and delay):
            delay = False

        """if (controller_1.buttonX.pressing()):
            clawP.set(True)
        elif (controller_1.buttonY.pressing()):
            clawP.set(False)"""
        wait(15, MSEC)


def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # place driver control in this while loop

    #prevents the loop from taking up all the brains resources.
    wait(15, MSEC)


    while True:
        thread1 = Thread(drive)
        thread2 = Thread(d4rb)
        #thread3 = Thread(intake)
        thread4 = Thread(claw)

        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()

motor_FL.spin(FORWARD)
motor_FR.spin(FORWARD)
motor_BL.spin(FORWARD)
motor_BR.spin(FORWARD)
motor_ML.spin(FORWARD)
motor_MR.spin(FORWARD)
