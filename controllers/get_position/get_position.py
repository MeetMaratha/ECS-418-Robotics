"""get_position controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Supervisor

class GetPositionOfEPuck():
    def __init__(self):
        self.sup = Supervisor()
        self.robot = sup.getFromDef('E-PUCK')
        self.translation_field = robot.getField('translation')
    
    def getpos(self):
        return self.translation_field.getSFVec3f() 



    
# create the Robot instance.


# get the time step of the current world.
# timestep = 32

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # pass

# Enter here exit cleanup code.
