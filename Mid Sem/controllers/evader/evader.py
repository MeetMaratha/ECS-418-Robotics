from controller import Supervisor
import numpy as np
import sys


# =================== GLOBALS ==========================
TIME_STEP = 32
ROBOT_NAME = "evader"
MAX_SPEED = 6.28

# =================== FUNCTIONS =======================

def _turnLeft90(robot, left_motor, right_motor):
    temp = 46
    while robot.step(TIME_STEP) != -1:
        left_speed = - MAX_SPEED * 0.25
        right_speed = MAX_SPEED * 0.25
        if temp == 0:
            break
        temp -= 1
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

def _turnRight90(robot, left_motor, right_motor):
    temp = 46
    while robot.step(TIME_STEP) != -1:
        left_speed = MAX_SPEED * 0.25
        right_speed = - MAX_SPEED * 0.25
        if temp == 0:
            break
        temp -= 1
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


# =================== MAIN FUCTION ====================

if __name__ == "__main__":
    
    #
    # DEFINING SUPERVISIOR AND ROBOT POINTER TO GET VALUES
    #
    robot = Supervisor()
    e_puck = robot.getFromDef(ROBOT_NAME)
    if robot is None:
        # There is no robot with the DEF as the one mentioned above
        sys.stderr.write(f"No DEF for {ROBOT_NAME} node found in the current world file\n")
        sys.exit()

    # ============== COMPONENTS INITALIZATION AND FIELD VALUE POINTERS =============

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    camera1 = robot.getDevice("camera 1")
    camera2 = robot.getDevice("camera 2")
    camera3 = robot.getDevice("camera 3")
    camera4 = robot.getDevice("camera 4")

    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    camera1.enable(TIME_STEP)
    camera2.enable(TIME_STEP)
    camera3.enable(TIME_STEP)
    camera4.enable(TIME_STEP)

    camera1.recognitionEnable(TIME_STEP)
    camera2.recognitionEnable(TIME_STEP)
    camera3.recognitionEnable(TIME_STEP)
    camera4.recognitionEnable(TIME_STEP)

    # ============== LOOP =================

    while robot.step(TIME_STEP) != -1:

        if np.random.rand() < 0.05 : 
            _turnLeft90(robot, left_motor, right_motor)
            left_speed, right_speed = 0.0, 0.0 
        elif np.random.rand() > 0.95 : 
            _turnRight90(robot, left_motor, right_motor)
            left_speed, right_speed = 0.0, 0.0
        else : 
            left_speed = MAX_SPEED * 0.25
            right_speed = MAX_SPEED * 0.25
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        # =============== CHECKING IF IT HAS BEEN FOUND ==============

        if camera1.getRecognitionNumberOfObjects() > 0 or camera2.getRecognitionNumberOfObjects() > 0 or camera3.getRecognitionNumberOfObjects() > 0 or camera4.getRecognitionNumberOfObjects() > 0:
            print(f"I have been captured :( ")
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            sys.exit()

        # ============= SETTING VELOCITY FOR THIS TIMESTEP ===========
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)




    