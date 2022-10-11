from controller import Supervisor
import numpy as np
import sys


# =================== GLOBALS ==========================
TIME_STEP = 32
SENSING_VALUE = 110 # 95
COUNTER = 0
GOAL=  [0,0,0] # For Failing of Bug2
# GOAL = [0.34, 0.67, 0]
ROBOT_NAME = "e-puck"
MAX_SPEED = 6.28
COLLISION = False
CHECKING_TIME = 10

# =================== FUNCTIONS =======================

def _getSlopeAndIntercept(point1 : list, point2 : list) -> tuple:
    """Returns slope and intercept calculated from 2 points

    Args:
        point1 (list): Point 1
        point2 (list): Point 2

    Returns:
        tuple: Slope and Intercept calculated from the equation
    """
    slope = ( point2[1] - point1[1] ) / ( point2[0] - point1[0] )
    intercept = ( point2[0] * point1[1] - point1[0] * point2[1] ) / ( point2[0] - point1[0] )
    return slope, intercept

def goToGoal(translation_field, rotation_field, max_speed : float, goal : list) -> tuple:
    """This dictates the go to goal procedure

    Args:
        translation_field (_type_): Translation field for the robot to get the position of the robot
        rotation_field (_type_): Rotation field for the robot to get the heading angle of the robot and axis-angle representation
        max_speed (float): MAX_SPEED of the robot
        goal (list): GOAL position

    Returns:
        TUPLE[float]: The left and right motor speed
    """
    position = translation_field.getSFVec3f()
    rot = rotation_field.getSFRotation()

    z_theta = rot[-2] * rot[-1]

    slope, _ = _getSlopeAndIntercept(position, goal)
    slope_theta = np.arctan( slope )

    # The desired angle changes based on the quadrant we are in, so make cases for it

    if position[0] > 0.0 and position[1] > 0.0 : desired_angle = slope_theta - 3.14 # For First Quadrant
    
    elif position[0] > 0.0 and position[1] < 0.0 : desired_angle = 3.14 + slope_theta # For Fourth Quadrant
    
    elif position[0] < 0 and position[1] < 0 : desired_angle = slope_theta # For Third Quadrant

    elif position[0] < 0.0 and position[1] > 0.0 : desired_angle = slope_theta # For Second Quadrant

    if np.abs(desired_angle - z_theta) > 1e-1 :
        # If the difference of desired angle and z_theta value is significant we need to rotate

        left_speed = - max_speed * 0.25
        right_speed = max_speed * 0.25

    else:
        # We are facing the goal so move forward

        left_speed = max_speed * 0.25
        right_speed = max_speed * 0.25
    return left_speed, right_speed

def _atGoal(point1 : list, point2 : list) -> bool:
    """Returns if the distance between two points is significant or not

    Args:
        point1 (list): Point 1
        point2 (list): Point 2

    Returns:
        bool: If the distance is significant it return False, else it returns True
    """
    if np.sqrt( (point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 ) < 1e-1 : return True
    else : return False

def _onLine(position : list, slope : float, intercept : float, goal : list) -> bool:
    """Returns whether a point is on line joining the goal and starting robot position

    Args:
        position (list): Current robot position
        slope (float): Slope of the line which was calculated at start
        intercept (float): Intercept of the line which was calculated at start
        goal (list): position of goal

    Returns:
        bool: Whether the robot lies on that line, if yes it returns True, else it returns False
    """
    current_slope, current_intercept = _getSlopeAndIntercept(position, goal)
    if np.abs(current_slope - slope) < 5e-1 and np.abs(current_intercept - intercept) < 5e-1 : return True # If the point is on line 
    else : return False

def _turnLeft90(robot, left_motor, right_motor):
    temp = 40
    while robot.step(TIME_STEP) != -1:
        left_speed = - MAX_SPEED * 0.25
        right_speed = MAX_SPEED * 0.25
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        if temp == 0 : break
        temp -= 1

def _turnRight90(robot, left_motor, right_motor):
    temp = 50
    while robot.step(TIME_STEP) != -1:
        left_speed = MAX_SPEED * 0.25
        right_speed = - MAX_SPEED * 0.25
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        if temp == 0 : break
        temp -= 1

def _turnLeft90Special(robot, left_motor, right_motor):
    temp = 60
    while robot.step(TIME_STEP) != -1:
        left_speed = - MAX_SPEED * 0.25
        right_speed = MAX_SPEED * 0.25
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        if temp == 0 : break
        temp -= 1

def _moveForward20(robot, left_motor, right_motor):
    temp = 20
    while robot.step(TIME_STEP) != -1:
        left_speed = MAX_SPEED * 0.25
        right_speed = MAX_SPEED * 0.25
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        if temp == 0 : break
        temp -= 1


# =================== MAIN FUCTION ====================

if __name__ == "__main__":
    
    print(f"WARNING : If you are using it for Failing of Bug 2 please uncomment the line with comment 'For Failing of of Bug2 ' and comment the line above it")
    
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
    translation_field = e_puck.getField("translation")
    rotation_field = e_puck.getField("rotation")

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    ps7 = robot.getDevice("ps7")
    ps0 = robot.getDevice("ps0")
    ps2 = robot.getDevice("ps2")

    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    ps0.enable(TIME_STEP)
    ps7.enable(TIME_STEP)
    ps2.enable(TIME_STEP)

    # ============== CALCULATING SLOPE AND INTERCEPT ============
    slope, intercept = _getSlopeAndIntercept(translation_field.getSFVec3f(), GOAL)


    # ============== LOOP =================

    while robot.step(TIME_STEP) != -1:
        ps0_value = ps0.getValue()
        ps7_value = ps7.getValue()
        ps2_value = ps2.getValue()

        # =============== COLLISION CHECKING ==============
        
        if (ps0_value >= SENSING_VALUE or ps7_value >= SENSING_VALUE) or (ps2_value >= SENSING_VALUE):
            COLLISION = True
            # COUNTER = 40
            COUNTER = 60 # For Failing of of Bug2 
        elif COUNTER == 0:
            COLLISION = False      
        else:
            COUNTER -= 1
            # COLLISION = False
            CHECKING_TIME -= 1
        if CHECKING_TIME < 0:
            if _onLine(translation_field.getSFVec3f(), slope, intercept, GOAL):
                COLLISION = False
                _turnLeft90Special(robot, left_motor, right_motor)
                _moveForward20(robot, left_motor, right_motor)
                COUNTER = 0
                CHECKING_TIME = 10
        else: CHECKING_TIME -= 1
        
        
        if (not COLLISION) and _onLine(translation_field.getSFVec3f(), slope, intercept, GOAL):
            CHECKING_TIME = 500 if CHECKING_TIME < 0 else CHECKING_TIME
            # If there is no collision go towards goal
            left_speed, right_speed = goToGoal(translation_field, rotation_field, MAX_SPEED, GOAL)
        
        elif COLLISION:
            CHECKING_TIME -= 1

            if (ps0_value >= SENSING_VALUE and ps7_value >= SENSING_VALUE) and ps2_value < SENSING_VALUE:
                _turnLeft90(robot, left_motor, right_motor)
            
            if ps0_value < SENSING_VALUE or ps7_value < SENSING_VALUE or ps2_value >= SENSING_VALUE:
                left_speed = MAX_SPEED * 0.25
                right_speed = MAX_SPEED * 0.25
            
            if ps0_value < SENSING_VALUE and ps7_value < SENSING_VALUE and ps2_value < SENSING_VALUE and COUNTER == 0:
                _turnRight90(robot, left_motor, right_motor)

        # ============= SETTING VELOCITY FOR THIS TIMESTEP ===========
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        if _atGoal(translation_field.getSFVec3f(), GOAL) : 
            # We have reached goal
            print(f"We have reached Goal !!!!")
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            sys.exit()
        



    