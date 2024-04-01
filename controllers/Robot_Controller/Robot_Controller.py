"""robot2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard, Motor, DistanceSensor

# create the Robot instance.
robot = Robot()
keyboard = Keyboard()

timestep = int(robot.getBasicTimeStep())
MAX_CAR_SPEED = 10
GRIP_SPEED = 0.01
ARM_SPEED = 0.5
NUM_MOTORS = 4
DEFAULT_SPEED = 0.5
MAX_GRIP_POSITION = 0.031
MIN_GIRP_POSITION = 0.066
DEFAULT_TURN_RATE = 10
full_turn_rate = DEFAULT_TURN_RATE
is_carrying = False
stop_arm_down = False


keyboard.enable(timestep)

# wheels motors
motors: list[Motor] = []
motor_names = ["m_backRight", "m_frontRight", "m_frontLeft", "m_backLeft"]

# infrared sensors
infrared_sensors = ["inf_front_right", "inf_front_left"]
inf_sensors: list[DistanceSensor] = []

# body distance sensor
body_ds = robot.getDevice("body_ds")
body_ds.enable(timestep)

# arm motor
arm_motor = robot.getDevice("arm_motor")
arm_motor.setPosition(float('inf'))
arm_motor.setVelocity(0)

# arm position sensor
arm_sensor = robot.getDevice("arm_ps")
arm_sensor.enable(timestep)

# left grip motor
left_grip_motor = robot.getDevice("left_grip_motor")
left_grip_motor.setPosition(float('inf'))
left_grip_motor.setVelocity(0)

# left grip ps
left_grip_ps = robot.getDevice("left_grip_ps")
left_grip_ps.enable(timestep)
# -0.019

# right grip motor
right_grip_motor = robot.getDevice("right_grip_motor")
right_grip_motor.setPosition(float('inf'))
right_grip_motor.setVelocity(0)


# right grip ps
right_grip_ps = robot.getDevice("right_grip_ps")
right_grip_ps.enable(timestep)
# 0.015

# grip distance sensor grip_ds
grip_ds = robot.getDevice("grip_ds")
grip_ds.enable(timestep)


for i in range(len(infrared_sensors)):
    inf_sensors.append(robot.getDevice(infrared_sensors[i]))
    inf_sensors[i].enable(timestep)


for i in range(NUM_MOTORS):
    motors.append(robot.getDevice(motor_names[i]))
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0)

def go_forward(power):
    for i in range(NUM_MOTORS):
        motors[i].setVelocity(power * MAX_CAR_SPEED)

def turn(power,direction):
    motors[0].setVelocity(power * MAX_CAR_SPEED * direction)
    motors[1].setVelocity(power * MAX_CAR_SPEED * direction)
    motors[2].setVelocity(-1 * power * MAX_CAR_SPEED * direction)
    motors[3].setVelocity(-1 * power * MAX_CAR_SPEED * direction)

def full_turn(power):
        global full_turn_rate
        if full_turn_rate > 0:
            full_turn_rate = full_turn_rate -1
            go_forward(power)

        if full_turn_rate == 0:
            turn(power,1)

def reset_turn_rate():
        global full_turn_rate
        if full_turn_rate < DEFAULT_TURN_RATE:
            full_turn_rate = DEFAULT_TURN_RATE

def arm_up(power = 1, callback = None):
    print("arm sensor: ", arm_sensor.getValue())
    arm_motor.setVelocity(ARM_SPEED * power)
    global is_carrying
    global left_grip_motor
    global right_grip_motor
    if arm_sensor.getValue() > 3.5:
        if callback is not None:
            callback()
        print("im here")
        arm_motor.setVelocity(0)
        if is_carrying == False:
            grips_stop()
            print("should not bere here")
            is_carrying = True
        # else:
        #     is_carrying = False
        


        


def arm_down(callback= None):
    arm_motor.setVelocity(-1 * ARM_SPEED)
    if(arm_sensor.getValue() < 1.6):
        # print("should stop now")
        arm_motor.setVelocity(0)
        if callback is not None:
            callback()

def grips_stop():
    left_grip_motor.setVelocity(0)
    right_grip_motor.setVelocity(0)


def grips_close(power):
    left_grip_motor.setVelocity(-1 * GRIP_SPEED * power)
    right_grip_motor.setVelocity(GRIP_SPEED * power)

def grips_open(power,callback=None):
        global stop_arm_down
    # if float(format(left_grip_ps.getValue(),'.3f')) > -1 * MAX_GRIP_POSITION or float(format(right_grip_ps.getValue(),'.3f')) < MAX_GRIP_POSITION:
        if float(format(left_grip_ps.getValue(),'.3f')) > -1 * MAX_GRIP_POSITION:
            left_grip_motor.setVelocity(0)
            # arm_motor.setVelocity(ARM_SPEED)
        else:
            left_grip_motor.setVelocity(GRIP_SPEED * power)

        if float(format(right_grip_ps.getValue(),'.3f')) < MAX_GRIP_POSITION:
            right_grip_motor.setVelocity(0)
            stop_arm_down = True
            # if callback is not None:
            #     callback()
            # arm_motor.setVelocity(ARM_SPEED)
        else:
            right_grip_motor.setVelocity(-1 * GRIP_SPEED * power)
        
        # if float(format(left_grip_ps.getValue(),'.3f')) < -1 * MAX_GRIP_POSITION or float(format(right_grip_ps.getValue(),'.3f')) > -1 * MAX_GRIP_POSITION:
        #     print("hereee")
            
           
    # else:
    #     print("hello")
    #     if callback is not None:
    #         callback()
    

def stop_and_close_grip_then_lift_up():
        grips_close(1)
        # it means that the grip is closed
        if float(format(left_grip_ps.getValue(),'.3f')) < -1 * MIN_GIRP_POSITION or float(format(right_grip_ps.getValue(),'.3f')) > MIN_GIRP_POSITION:
            # print("grip is closed")
            # grip velocity should be 0 now
            # arm should be up now
            arm_up(2)
            # pass  
    
    

def open_grips_and_arm_up():
        grips_open(1)
        # arm_up(1)

def change_stop_arm_down(val = False):
    global stop_arm_down
    stop_arm_down = val

def stop_and_lift_down_then_open_grip_then_arm_up():
    if stop_arm_down == False:
        grips_close(1)
        arm_down(open_grips_and_arm_up)
    elif stop_arm_down == True:
        arm_up(1)
    # if float(format(left_grip_ps.getValue(),'.3f')) < -0.047 or float(format(right_grip_ps.getValue(),'.3f')) > 0.049:
    #     print("here")
    #     arm_down()
    # # it means that the arm is down
    # if arm_sensor.getValue() < 1.6:
    #     print("hereeee")
        # grips_open(1)
        # arm_up()
    # # it means that the grip is open and the arm is down
    # grip_side_stop()
    
       
    

while robot.step(timestep) != -1:
    # print("grip_ds: ", grip_ds.getValue())
    # print("body_ds: ", body_ds.getValue())
    # print("is_carrying: ", is_carrying)
    # print("left_grip_ps: ", format(left_grip_ps.getValue(),'.3f'))
    # print("right_grip_ps: ", format(right_grip_ps.getValue(),'0.3f'))
    # print("arm sensor: ", arm_sensor.getValue())
    
    # right_grip_motor.setVelocity(-1 *GRIP_SPEED)
    # left_grip_motor.setVelocity(GRIP_SPEED)

    # turn right if obstacle on the right
    if inf_sensors[0].getValue() < 900 and inf_sensors[1].getValue() < 900:
        full_turn(DEFAULT_SPEED)
        print("full turn")
    elif inf_sensors[0].getValue() < 900 and inf_sensors[1].getValue() > 900:
        print("turn right")
        turn(DEFAULT_SPEED,-1)
        reset_turn_rate()
    # turn left if obstacle on the left
    elif inf_sensors[1].getValue() < 900 and inf_sensors[0].getValue() > 900:
        print("turn left")
        turn(DEFAULT_SPEED,1)
        reset_turn_rate()
    elif is_carrying == False and grip_ds.getValue() < 1000:
        # print("pull up")
        go_forward(0)
        stop_and_close_grip_then_lift_up()
    elif is_carrying == True and body_ds.getValue() < 300:
        print("put down")
        go_forward(0)
        stop_and_lift_down_then_open_grip_then_arm_up()
    else:
        print("forward")
        go_forward(DEFAULT_SPEED)

        
    pass


# Enter here exit cleanup code.


# keyboard control
# key = keyboard.getKey()
# if key != -1:
#     print(key)
# if key == keyboard.UP:
#     go_forward(DEFAULT_SPEED)
# elif key == keyboard.DOWN:
#     go_forward(-DEFAULT_SPEED)
# elif key == keyboard.LEFT:
#     turn(DEFAULT_SPEED,-1)
# elif key == keyboard.RIGHT:
#     turn(DEFAULT_SPEED,1)
# else:
#     go_forward(0)
    # turn(0,0)