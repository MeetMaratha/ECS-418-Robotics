from controller import Supervisor
import numpy as np
import sys

####################################################
# GLOBALS
####################################################
TIME_STEP = 32
SENSING_VALUE = 115
COUNTER = 0
GOAL = [(-3.49, 0.42, 0), (0.45, -3.73, 0)]
MAX_SPEED = 6.28
COLLISION = False
TIME = 0
ROBOT_NAME = 'obs1'
GOAL_IDX = 1

####################################################
# CUSTOM FUNCTIONS
####################################################
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
    desired_angle = np.arctan2( goal[1] - position[1], goal[0] - position[0] )
    # print(f"Robot Angle : {rot[-1]} | Desired Angle : {desired_angle} | Checking Angle : {np.abs(rot[-1] * rot[-2] - desired_angle * rot[-2])}")
    if np.abs(z_theta - desired_angle ) > 9e-2:
        left_speed = -max_speed * 0.25
        right_speed = max_speed * 0.25
    else:
        left_speed = max_speed
        right_speed = max_speed
    
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


####################################################
# MAIN FUNCTION
####################################################
if __name__ == "__main__":
    
    ####################################################
    # DEFINING SUPERVISIOR AND ROBOT POINTER TO GET VALUES
    ####################################################
    robot = Supervisor()
    e_puck = robot.getFromDef(ROBOT_NAME)
    if robot is None:
        # There is no robot with the DEF as the one mentioned above
        sys.stderr.write(f"No DEF for {ROBOT_NAME} node found in the current world file\n")
        sys.exit()
    
    ####################################################
    # COMPONENTS INITALIZATION AND FIELD VALUE POINTERS 
    ####################################################
    translation_field = e_puck.getField("translation")
    rotation_field = e_puck.getField("rotation")

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    ####################################################
    # SETTING GOAL
    ####################################################
    goal = GOAL[GOAL_IDX]
    
    ####################################################
    # MAIN LOOP
    ####################################################
    while robot.step(TIME_STEP) != -1:
        
        ####################################################
        # IF NO COLLISION, GO TO GOAL
        ####################################################
        left_speed, right_speed = goToGoal(translation_field, rotation_field, MAX_SPEED, goal)
        
        ####################################################
        # ACTION IF REACHED GOAL
        ####################################################        
        if _atGoal(translation_field.getSFVec3f(), goal):
            GOAL_IDX = (GOAL_IDX + 1) % len(GOAL)
            goal = GOAL[GOAL_IDX]
            
        ####################################################
        # SETTING VELOCITY FOR THIS TIMESTEP
        ####################################################        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        TIME = TIME + TIME_STEP




    