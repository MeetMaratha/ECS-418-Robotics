from controller import Supervisor
import numpy as np

SENSING_RANGE = 115

def distance(p1, p2):
    return np.sqrt( (p2[0] - p1[0]) ** 2 + (p2[1] - p2[0]) ** 2 )

def rotate(rot, pos, goal, translation_field, rotation_field, sup, timestep, left_motor, right_motor):
    if goal[0] > pos[0] and goal[1] > pos[1]:
        # Turn Left
        left_speed = max_speed * 0.25 if rot[2] > 0 else -max_speed * 0.25
        right_speed = -max_speed * 0.25 if rot[2] > 0 else max_speed * 0.25
    elif goal[0] > pos[0] and goal[1] < pos[1]:
        # Turn Right
        left_speed = -max_speed * 0.25 if rot[2] > 0 else max_speed * 0.25
        right_speed = max_speed * 0.25 if rot[2] > 0 else -max_speed * 0.25
    elif goal[0] < pos[0] and goal[1] < pos[1]:
        # Turn Right
        left_speed = max_speed * 0.25 if rot[2] < 0 else -max_speed * 0.25
        right_speed = -max_speed * 0.25 if rot[2] < 0 else max_speed * 0.25
    elif goal[0] < pos[0] and goal[1] > pos[1]:
        # Turn Left
        left_speed = -max_speed * 0.25 if rot[2] < 0 else max_speed * 0.25
        right_speed = max_speed * 0.25 if rot[2] < 0 else -max_speed * 0.25
    slope = (goal[1] - pos[1])/(goal[0] - pos[0])
    m1, m2 = slope, np.tan(rot[-1])
    angle_to_rotate = np.arctan( (m2 - m1)/(1 + m1*m2) )
    times = np.abs(angle_to_rotate) // 0.03
    while sup.step(timestep) != -1:
        pos = translation_field.getSFVec3f()
        
        # if np.abs(angle_to_rotate) < 5e-2:
            # break
        if times == 0:
            break
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        # print(f"Angle to Rotate : {angle_to_rotate}")
        times -= 1
            
def goToGoal(translation_field, rotation_field, max_speed, goal, sup, timestep, left_motor, right_motor):
    pos = translation_field.getSFVec3f()
    rot = rotation_field.getSFRotation()
    
    slope = (goal[1] - pos[1])/(goal[0] - pos[0])
    m1, m2 = slope, np.tan(rot[-1])
    angle_to_rotate = np.arctan( (m2 - m1)/(1 + m1*m2) )
    
    if np.abs(angle_to_rotate) > 5e-1: # e-2
        # This works
        # if angle_to_rotate < 0:
            # left_speed = -max_speed * 0.25
            # right_speed = max_speed * 0.25
        # elif angle_to_rotate > 0:
            # left_speed = max_speed * 0.25
            # right_speed = -max_speed * 0.25
            
        # To bring it back on track if it goes astray
        # if pos[1] > 0.0 and rot[-1] > 0 and pos[0] < 0:
            # rotation_field.setSFRotation([0, 0, 1, -0.737])
        # if pos[1] > 0.0 and rot[-1] > -1.57 and pos[0] > 0:
            # rotation_field.setSFRotation([0, 0, 1, -2.62])
        # if pos[1] < 0.0 and (rot[-1] < 0 or rot[-1] > 2.5) and pos[0] > 0:
            # rotation_field.setSFRotation([0, 0, 1, 2.48])
        # if pos[1] < 0.0 and (rot[-1] < 0 or rot[-1] > 2.1) and pos[0] < 0:
            # rotation_field.setSFRotation([0, 0, 1, 1.06])
        
        # Experimentation
        rotate(rot, pos, goal, translation_field, rotation_field, sup, timestep, left_motor, right_motor)
        
        
        
    # else:
        # Move Forward
    left_speed = max_speed * 0.25
    right_speed = max_speed * 0.25
    print(slope, angle_to_rotate)
    return left_speed, right_speed, angle_to_rotate
    
# def goToGoal(translation_field, rotation_field, count_wrong_direction = 0):
    # pos = translation_field.getSFVec3f()
    # rot = rotation_field.getSFRotation()
    
    # slope = (goal[1] - pos[1]) / (goal[0] - pos[0]) # Get Slope between robot and goal
    
    # m2, m1 = np.tan(rot[-1]), slope
    
    # angle_to_rotate = np.arctan((m1 - m2)/(1 + m1 * m2)) # Angle to Rotate the robot
    # rotation_matrix = None
    # if np.abs(angle_to_rotate) > 5e-2  : # If angle is significant, rotate it
        # if slope > 0 and rot[-1] > 0:
            # rotation_matrix = np.array([0.0, 0.0, 1.0, rot[3] + angle_to_rotate]) if np.abs(rot[3]) < 1.57 else np.array([0.0, 0.0, 1.0, rot[3] - angle_to_rotate + 3.14])  #Rotation Matrix 
        # else:
            # rotation_matrix = np.array([0.0, 0.0, 1.0, rot[3] + angle_to_rotate]) if rot[3] < 1.57 else np.array([0.0, 0.0, 1.0, rot[3] + 0.1])
        # rotation_field.setSFRotation(list(rotation_matrix))    # Rotate Robot
        # old_position = pos
        # new_position = translation_field.getSFVec3f()
    # print(f"Robot : {rot[-3]} | Slope : {np.arctan(slope)} | Angle : {angle_to_rotate} | Rotation Matrix : {rotation_matrix} | m1 - m2 : {m1 - m2}")
    
    # if abs(m1 - m2)< 1e-4:
        # old_position, new_position = pos, translation_field.getSFVec3f()
        # if count_wrong_direction > 50:
            # rotation_matrix = np.array([0.0, 0.0, 1.0, rot[3] + 3.14])
            # rotation_field.setSFRotation(list(rotation_matrix))
            # count_wrong_direction = 0
        # print(count_wrong_direction)
        # print(f"Robot : {rot[-3]} | Slope : {np.arctan(slope)} | m1 - m2 : {m1 - m2}")
    
    # if np.abs(angle_to_rotate) < 1e-1 and m1 - m2 > 1e-4:
        # old_position = pos
        # new_position = pos
    # return count_wrong_direction, old_position, new_position
        

timestep = int(32)
max_speed = 6.28

sup = Supervisor()
robot = sup.getFromDef('robot')
goal = [0, 0, 0]

# Initalize Motors
left_motor = sup.getDevice('left wheel motor')
right_motor = sup.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Create Position Variable
translation_field = robot.getField('translation')
rotation_field = robot.getField('rotation')


# Initalize Distance Sensors
ps0 = sup.getDevice('ps1')   
ps0.enable(timestep)  
ps7 = sup.getDevice('ps7')
ps7.enable(timestep)
ps2 = sup.getDevice('ps2')
ps2.enable(timestep)
ps3 = sup.getDevice('ps3')
ps3.enable(timestep)
collision = False
counter = 0
t =2
last_distance = distance(translation_field.getSFVec3f()[:2], goal[:2])
while sup.step(timestep) != -1:
    
    ps0_value = ps0.getValue()
    ps7_value = ps7.getValue()
    ps2_value = ps2.getValue()
    ps3_value = ps3.getValue()
    new_position = translation_field.getSFVec3f()
    if (ps0_value >= SENSING_RANGE or ps7_value >= SENSING_RANGE or ps2_value >= SENSING_RANGE): 
        collision = True
        counter = 200
    elif counter == 0: 
        collision = False
    else:
        counter -= 1
        
    if not collision:
        left_speed, right_speed, prev_angle = goToGoal(translation_field, rotation_field, max_speed, goal, sup, timestep, left_motor, right_motor)
    # 46 Time step to rotat 90 degree
    elif collision:
        if ps0_value >= SENSING_RANGE or ps7_value >= SENSING_RANGE:
        # Collision in forward, turn left
            left_speed = -max_speed * 0.25
            right_speed = max_speed * 0.25
        elif ps2_value >= SENSING_RANGE and ps0_value < SENSING_RANGE and ps7_value < SENSING_RANGE:
            # Obstacle on right, move forward to follow the obstacle
            left_speed = max_speed * 0.25
            right_speed = max_speed * 0.25
        elif ps0_value < SENSING_RANGE and ps7_value < SENSING_RANGE and ps2_value < SENSING_RANGE and counter < 150 and counter > 105:
            left_speed = max_speed * 0.25
            right_speed = -max_speed * 0.25
        elif ps0_value < SENSING_RANGE and ps7_value < SENSING_RANGE and ps2_value < SENSING_RANGE and counter <= 105:
            left_speed = max_speed * 0.25
            right_speed = max_speed * 0.25
            
    
        
    # print(f"Counter : {counter}, t : {t}")
    
    # print("ps0: {} ps7: {} ps2: {} counter: {} t: {}".format(ps0_value, ps7_value, ps2_value, counter, t))
    
    
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    rot = rotation_field.getSFRotation()
    pos = translation_field.getSFVec3f()
    # Extras
    if distance(translation_field.getSFVec3f()[:2], goal) > last_distance and not collision:
        if t == 0:
            # rotation_field.setSFRotation([0, 0, 1, rot[-1] + 3.14])
            if pos[1] > 0.0 and rot[-1] > 0 and pos[0] < 0:
                rotation_field.setSFRotation([0, 0, 1, -0.737])
            # if pos[1] > 0.0 and rot[-1] > -1.57 and pos[0] > 0:
                # rotation_field.setSFRotation([0, 0, 1, -2.62])
            t = 2
        if pos[0] < goal[0] and pos[1] > goal[1]:
            t -= 1
        if pos[0] > goal[0] and pos[1] > goal[1]:
            t -= 1
    last_distance = distance(translation_field.getSFVec3f()[:2], goal)
    