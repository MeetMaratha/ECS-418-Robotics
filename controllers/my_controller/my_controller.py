from controller import Robot, DistanceSensor, Motor
# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()
# robot1 = Node.getFromDef("e-puck");

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
ls = robot.getDevice('light sensor')
ls.enable(TIME_STEP)
# feedback loop: step simulation until receiving an exit event
counter = 0
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    print(ls.getValue())
    # detect obstacles
    right_obstacle = psValues[0] >= 100.0
    left_obstacle = psValues[7] >= 100.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0 * MAX_SPEED
        rightSpeed = 0.25 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.25 * MAX_SPEED
        rightSpeed = 0.0 * MAX_SPEED
    elif right_obstacle and left_obstacle:
        if counter < 10:
            left_speed = 0.5*MAX_SPEED
            right_speed = 0*MAX_SPEED
        else:
            counter = 0
            left_speed = 0.5*MAX_SPEED
            right_speed = 0.5*MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)