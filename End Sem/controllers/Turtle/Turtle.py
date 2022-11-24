from controller import Supervisor
import numpy as np
import sys
TIMESTEP = 32
BASE_SPEED = 5
WAYPOINT = [(-0.713, 1.325), (-0.173, 1.325), (0.117, 1.325), (0.7, 1.30), (0.7, -1.23)]

def distanceBetweenTwoPoints(point1 : tuple, point2 : tuple) -> float:
    """Gives euclidean distance between two points

    Args:
        point1 (tuple): Cartesian co-ordinate of 1st Point
        point2 (tuple): Cartesian co-ordinate of 2nd Point

    Returns:
        float: Euclidean distance between those two points
    """
    return np.sqrt( (point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2 )

def findFoot(line_points : tuple, robot : tuple) -> tuple:
    """Reutrns point of Foot of the perpendicular to the line ax + by + c = 0 from point (x1, y1)

    Args:
        a (float): Coefficent of x in line equation
        b (float): Coefficent of y in line equation
        c (float): Line intercept
        x1 (float): X-Co-oridinate of the point from which perpendicular is drawn
        y1 (float): Y-Co-oridinate of the point from which perpendicular is drawn

    Returns:
        tuple: The co-ordinate of the foot of the perpendicular
    """
    
    x1, y1 = robot
    assert(len(line_points)) == 2
    assert(len(robot)) == 2
    for p in line_points : assert(len(p)) == 2
    p1, p2 = line_points
    a = (p2[1] - p1[1])/(p2[0] - p1[0])
    c = (p1[0] * p2[1] - p2[0] * p1[1])/(p1[0] - p2[0])
    b = -1
    temp = (-1 * (a * x1 + b * y1 + c) / (a * a + b * b))
    x = temp * a + x1
    y = temp * b + y1
    return (x, y)

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
        if goal[0] - position[0] < 0.0:
            left_speed = -max_speed 
            right_speed = max_speed 
        else:
            left_speed = max_speed
            right_speed = -max_speed
    else:
        left_speed = max_speed
        right_speed = max_speed
    
    return left_speed, right_speed

def _distanceBetweenLineAndPoint(point : list, line_points : list) -> float:
    """Returns distance between a point and a line

    Args:
        point (list | tuple): point from which we need the distance
        line_points (list[list] | list[tuple] | tuple[tuple] | tuple[list]): two points which makes the line

    Returns:
        float: distance between the point and the line
    """
    assert(len(line_points)) == 2
    assert(len(point)) == 2
    for p in line_points : assert(len(p)) == 2
    p1, p2 = line_points
    m = (p2[1] - p1[1])/(p2[0] - p1[0])
    b = (p1[0] * p2[1] - p2[0] * p1[1])/(p1[0] - p2[0])
    return np.abs(- m*point[0] + point[1] - b)/np.sqrt(m**2+1)

def intializeTurtle(robot : Supervisor) -> tuple:
    """Initalize the turtle bot

    Args:
        robot (Supervisor): The supervisor instance of robot

    Returns:
        tuple: Motors variables
    """

    ###########################################################
    # MOTORS
    ###########################################################
    left_wheel_motor = robot.getDevice('left wheel motor')
    right_wheel_motor = robot.getDevice('right wheel motor')
    
    left_wheel_motor.setPosition(float('inf'))
    right_wheel_motor.setPosition(float('inf'))
    
    left_wheel_motor.setVelocity(1.0)
    right_wheel_motor.setVelocity(1.0)
    
    return left_wheel_motor, right_wheel_motor

def runRobot(robot, left_motor, right_motor, tr_cr0, tr_cr1, tr_r, rf_r) -> None:
    """Main function which runs the robot

    Args:
        robot (_type_): Robot instance of supervisor class
        left_motor (_type_): Robot left motor instance
        right_motor (_type_): Robot right motor instance
        tr_cr0 (_type_): Translation field instance of obstacle 1
        tr_cr1 (_type_): Translation field instance of obstacle 2
        tr_r (_type_): Translation field instance of main robot
        rf_r (_type_): Rotation Field instance of main robot
    """

    p_cr0 = []
    p_cr1 = []
    goal_idx = 0
    goal = WAYPOINT[goal_idx]
    while robot.step(TIMESTEP) != -1:
        p_cr0.append(tr_cr0.getSFVec3f()[:2])
        p_cr1.append(tr_cr1.getSFVec3f()[:2])
        robot_pos = tr_r.getSFVec3f()[:2]
        if distanceBetweenTwoPoints(robot_pos, WAYPOINT[(goal_idx + 1) % len(WAYPOINT)]) < distanceBetweenTwoPoints(robot_pos, goal):
            goal_idx = goal_idx + 1
            goal = WAYPOINT[goal_idx]
            print(f"New Goal is {goal}.")
        if distanceBetweenTwoPoints(goal, robot_pos) < 1e-2:
            # Meaning robot has reached its waypoint
            goal_idx += 1
            if goal_idx < len(WAYPOINT):
                # Meaning we have not reached end goal
                goal = WAYPOINT[goal_idx]
                print(f"New Goal is {goal}.")
            else:
                # We have reached end goal
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                print("Reached End!!!")
                sys.exit()
        if len(p_cr0) < 2 : 
            left_speed, right_speed = BASE_SPEED, BASE_SPEED
        if len(p_cr0) >= 2 and robot_pos[0] <=0:
            # In first block
            dist0 = _distanceBetweenLineAndPoint(robot_pos, p_cr0[-2:])
            collision = True if dist0 < 0.5 and (distanceBetweenTwoPoints(robot_pos, p_cr0[-1]) - distanceBetweenTwoPoints(robot_pos, p_cr0[-2])) < 0.0 and distanceBetweenTwoPoints(robot_pos, p_cr0[-1]) < 2 else False
            if collision:
                
                foot = findFoot(p_cr0[-2:], robot_pos)
                if foot[0] - robot_pos[0] > 0 : 
                    # Foot is on the right
                    left_speed = BASE_SPEED * 0.5
                    right_speed = BASE_SPEED 
                else:
                    # Foot is on the left
                    left_speed = BASE_SPEED 
                    right_speed = BASE_SPEED* 0.5
            else:
                left_speed, right_speed = goToGoal(tr_r, rf_r, BASE_SPEED, goal)
        elif len(p_cr1) >= 2 and robot_pos[0] > 0:
            # In first block
            dist1 = _distanceBetweenLineAndPoint(robot_pos, p_cr1[-2:])
            collision = True if dist1 < 0.5 and (distanceBetweenTwoPoints(robot_pos, p_cr1[-1]) - distanceBetweenTwoPoints(robot_pos, p_cr1[-2])) < 0.0 and distanceBetweenTwoPoints(robot_pos, p_cr1[-1]) < 2 else False
            if collision:
                foot = findFoot(p_cr1[-2:], robot_pos)
                if foot[0] - robot_pos[0] < 0 : 
                    # Foot is on the right
                    left_speed = BASE_SPEED
                    right_speed = BASE_SPEED * 0.5
                else:
                    # Foot is on the left
                    left_speed = BASE_SPEED * 0.5
                    right_speed = BASE_SPEED
            else:
                left_speed, right_speed = goToGoal(tr_r, rf_r, BASE_SPEED, goal)
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
            
if __name__ == "__main__":
    robot = Supervisor()
    cr0 = robot.getFromDef("Create0")
    cr1 = robot.getFromDef("Create1")
    r = robot.getFromDef("Turtle")
    
    tr_cr0 = cr0.getField("translation")
    tr_cr1 = cr1.getField("translation")
    tr_r = r.getField("translation")
    rf_r = r.getField("rotation")
    
    left_motor, right_motor = intializeTurtle(robot)
    timestep = int(robot.getBasicTimeStep())
    
    runRobot(robot, left_motor, right_motor, tr_cr0, tr_cr1, tr_r, rf_r)
