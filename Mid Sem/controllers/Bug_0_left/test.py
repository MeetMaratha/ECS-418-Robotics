"""cont1 controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Supervisor
import sys
import numpy as np
TIME_STEP=32
range=115
counter=0
supervisor = Supervisor()
# create the Robot instance.
# robot = Robot()
goal=[-0.427, 0.456, 0]
robot_node = supervisor.getFromDef("e-puck")
if robot_node is None:
    sys.stderr.write("No DEF e-puck node found in the current world file\n")
    sys.exit(1)
maxSpeed=6.28
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")
left_motor=supervisor.getDevice("left wheel motor")
right_motor=supervisor.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
ps7=supervisor.getDevice("ps7")
ps5=supervisor.getDevice("ps5")
ps0=supervisor.getDevice("ps0")
ps2=supervisor.getDevice("ps2")
ps7.enable(TIME_STEP)
ps5.enable(TIME_STEP)
ps0.enable(TIME_STEP)
ps2.enable(TIME_STEP)
gps=supervisor.getDevice("gps1")
gps.enable(TIME_STEP)

def goToGoal(trans_field, rot_field, maxSpeed, goal, supervisor): 
    values = trans_field.getSFVec3f()
    v_rot=rot_field.getSFRotation()
    zt=v_rot[2]*(v_rot[3])
    
    slope = (np.arctan((goal[1] - values[1])/(goal[0] - values[0])))
    if (values[0]<goal[0] and values[1]<=goal[1]):
        # slope=slope-3.14
        to_rot=slope
        print(to_rot, zt)
        if (np.abs(to_rot-zt)>1e-1):
            left_speed=-maxSpeed*0.25
            right_speed=maxSpeed*0.25
        else:
            left_speed=maxSpeed*0.5
            right_speed=maxSpeed*0.5
        return left_speed,right_speed
    elif (values[0]<goal[0] and values[1]>goal[1]):
        print(slope)
        # slope=3.14+slope
        to_rot=slope
        print(to_rot, zt)
        if (np.abs(to_rot-zt)>1e-1):
            left_speed=-maxSpeed*0.25
            right_speed=maxSpeed*0.25
        else:
            left_speed=maxSpeed*0.5
            right_speed=maxSpeed*0.5
        return left_speed,right_speed
    elif (values[0]>goal[0] and values[1]>goal[1]):
        to_rot=slope-3.14
        print(to_rot, zt)
        if (np.abs(to_rot-zt)>1e-1):
            left_speed=-maxSpeed*0.25
            right_speed=maxSpeed*0.25
        else:
            left_speed=maxSpeed*0.5
            right_speed=maxSpeed*0.5
        return left_speed,right_speed
    elif (values[0]>goal[0] and values[1]<goal[1]):
        to_rot=3.14+slope
        print(to_rot, zt)
        if (np.abs(to_rot-zt)>1e-1):
            left_speed=-maxSpeed*0.25
            right_speed=maxSpeed*0.25
        else:
            left_speed=maxSpeed*0.5
            right_speed=maxSpeed*0.5
        return left_speed,right_speed
    # to_rot=((3.14-np.abs(slope))) if values[1] <= 0.0 else -((3.14-np.abs(slope)))
    # print(np.abs(np.abs(v_rot[-2]*to_rot)-zt)-3.14, zt, slope)
    # if (np.abs(v_rot[-2]*to_rot-zt)>1e-1):
        # left_speed=-maxSpeed*0.25
        # right_speed=maxSpeed*0.25
    # else:
        # left_speed=maxSpeed*0.5
        # right_speed=maxSpeed*0.5
    # return left_speed,right_speed
        
    
while supervisor.step(TIME_STEP) != -1:
    # this is done repeatedly
    # values = trans_field.getSFVec3f()
    # print("e-puck is at position: %g %g %g" % (values[0], values[1], values[2]))
    # v_rot=rot_field.getSFRotation()
    # print("e-puck is at rotation: %g %g %g %g" % (v_rot[0], v_rot[1], v_rot[2], v_rot[3]))
    # left_speed=maxSpeed*0.25
    # right_speed=maxSpeed*0.25
    # left_motor.setVelocity(left_speed)
    # right_motor.setVelocity(right_speed)
    ps0_value = ps0.getValue()
    ps7_value = ps7.getValue()
    ps2_value = ps2.getValue()
    ps5_value = ps5.getValue()
    new_position = trans_field.getSFVec3f()
    if (ps0_value >= range or ps7_value >= range or ps2_value >= range): 
        collision = True
        counter = 200
    elif counter == 0: 
        collision = False
    else:
        counter -= 1
        
    if not collision:
        left_speed, right_speed = goToGoal(trans_field, rot_field, maxSpeed, goal, supervisor)
    # 46 Time step to rotat 90 degree
    elif collision:
        if ps0_value >= range or ps7_value >= range:
        # Collision in forward, turn left
            left_speed = -maxSpeed * 0.25
            right_speed = maxSpeed * 0.25
        elif ps2_value >= range and ps0_value < range and ps7_value < range:
            # Obstacle on right, move forward to follow the obstacle
            left_speed = maxSpeed * 0.25
            right_speed = maxSpeed * 0.25
        elif ps0_value < range and ps7_value < range and ps2_value < range and counter < 150 and counter > 130:
            left_speed = maxSpeed * 0.25
            right_speed = -maxSpeed * 0.25
        elif ps0_value < range and ps7_value < range and ps2_value < range and counter <= 130:
            left_speed = maxSpeed * 0.25
            right_speed = maxSpeed * 0.25
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)