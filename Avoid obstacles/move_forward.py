from controller import Robot, DistanceSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'Lsensor', 'Rsensor'
]

for i in range(2):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftBackMotor = robot.getDevice('LBmotor')
leftFrontMotor = robot.getDevice('LFmotor')
rightBackMotor = robot.getDevice('RBmotor')
rightFrontMotor = robot.getDevice('RFmotor')

leftBackMotor.setPosition(float('inf'))
leftFrontMotor.setPosition(float('inf'))
rightBackMotor.setPosition(float('inf'))
rightFrontMotor.setPosition(float('inf'))
leftBackMotor.setVelocity(0.0)
leftFrontMotor.setVelocity(0.0)
rightBackMotor.setVelocity(0.0)
rightFrontMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(2):
        psValues.append(ps[i].getValue())
        
    # detect obstacles
    Rsensor = psValues[1] < 1000.0 
    Lsensor = psValues[0] < 1000.0 
    
    # initialize motor speeds at 50% of MAX_SPEED.
    LBSpeed  = 0.5 * MAX_SPEED
    LFSpeed  = 0.5 * MAX_SPEED
    RBSpeed = 0.5 * MAX_SPEED
    RFSpeed = 0.5 * MAX_SPEED
    
    # modify speeds according to obstacles
    if Lsensor:
        # turn right
        LBSpeed  = 0.5 * MAX_SPEED
        RBSpeed = -0.5 * MAX_SPEED
        LFSpeed  = 0.5 * MAX_SPEED
        RFSpeed = -0.5 * MAX_SPEED
    elif Rsensor:
        # turn left
        LBSpeed  = -0.5 * MAX_SPEED
        RBSpeed = 0.5 * MAX_SPEED
        LFSpeed  = -0.5 * MAX_SPEED
        RFSpeed = 0.5 * MAX_SPEED
        
    # write actuators inputs
    leftBackMotor.setVelocity(LBSpeed)
    leftFrontMotor.setVelocity(LFSpeed)
    rightBackMotor.setVelocity(RBSpeed)
    rightFrontMotor.setVelocity(RFSpeed)
