from controller import Supervisor
import numpy as np
import sys


# =================== GLOBALS ==========================
TIME_STEP = 32
SENSING_VALUE = 100
COUNTER = 0
GOAL = [-3.13, -2.8785, 0]
ROBOT_NAME = "e-puck"
MAX_SPEED = 6.28
COLLISION = False

# =================== FUNCTIONS =======================

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

    slope = ( position[1] - goal[1] ) / ( position[0] - goal[0] )
    slope_theta = np.arctan( slope )

    # The desired angle changes based on the quadrant we are in, so make cases for it

    if position[0] > 0.0 and position[1] > 0.0 : desired_angle = slope_theta - 3.14 # For First Quadrant
    
    elif position[0] > 0.0 and position[1] < 0.0 : desired_angle = 3.14 + slope_theta # For Fourth Quadrant
    
    elif position[0] < 0.0 and position[1] < 0.0 : desired_angle = slope_theta # For Third Quadrant

    elif position[0] < 0.0 and position[1] > 0.0 : desired_angle = slope_theta # For Second Quadrant

    if np.abs(desired_angle - z_theta) > 1e-1 :
        # If the difference of desired angle and z_theta value is significant we need to rotate

        left_speed = - MAX_SPEED * 0.25
        right_speed = MAX_SPEED * 0.25

    else:
        # We are facing the goal so move forward

        left_speed = MAX_SPEED * 0.25
        right_speed = MAX_SPEED * 0.25
    
    return left_speed, right_speed


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

    # ============== LOOP =================

    while robot.step(TIME_STEP) != -1:
        ps0_value = ps0.getValue()
        ps7_value = ps7.getValue()
        ps2_value = ps2.getValue()

        # =============== COLLISION CHECKING ==============
        
        if ps0_value >= SENSING_VALUE or ps7_value >= SENSING_VALUE or ps2_value >= SENSING_VALUE:
            COLLISION = True
            COUNTER = 100
        elif COUNTER == 0:
            COLLISION = False
        else:
            COUNTER -= 1
        
        if not COLLISION:
            # If there is no collision go towards goal
            left_speed, right_speed = goToGoal(translation_field, rotation_field, MAX_SPEED, GOAL)
        
        elif COLLISION:
            # If there is a collision follow the obstacle

            if ps0_value >= SENSING_VALUE or ps7_value >= SENSING_VALUE:
                # If the collision is in the forward direction turn left
                
                left_speed = - MAX_SPEED * 0.25
                right_speed = MAX_SPEED * 0.25
            
            elif ps2_value >= SENSING_VALUE and ps0_value < SENSING_VALUE and ps7_value < SENSING_VALUE:
                # We have moved in left and facing perpendicular to obstacle face, so move forward
                
                left_speed = MAX_SPEED * 0.25
                right_speed = MAX_SPEED * 0.25
            
            elif ps0_value < SENSING_VALUE and ps7_value < SENSING_VALUE and ps2_value < SENSING_VALUE and COUNTER < 50 and COUNTER > 30:
                # We have left the obstacle and moved a bit forward, so we now need to turn a bit to right
                
                left_speed = MAX_SPEED * 0.25
                right_speed = - MAX_SPEED * 0.25
            
            elif ps0_value < SENSING_VALUE and ps7_value < SENSING_VALUE and ps2_value < SENSING_VALUE and COUNTER <= 30:
                # Move forward a bit after turning a bit
                left_speed = MAX_SPEED * 0.25
                right_speed = MAX_SPEED * 0.25
    
        # ============= SETTING VELOCITY FOR THIS TIMESTEP ===========
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        print(GOAL)




    