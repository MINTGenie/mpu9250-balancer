def PID(input2: number):
    global timeChange, distance_from_goal, accumulative_error, change_in_error, last_error, Actuator_Output, lastInput, lastTime
    # in milliseconds
    now = game.current_time()
    # convert to seconds
    timeChange = (now - lastTime) / 1000
    # //  PID algorithm
    # Actuator_Output =
    # Kp * (distance from goal)
    # + Ki * (accumulative error)
    # + Kd * (change in error)
    # 
    # P
    distance_from_goal = setPoint - input2
    # I
    accumulative_error = accumulative_error + distance_from_goal * timeChange
    # D
    change_in_error = (distance_from_goal - last_error) / timeChange
    last_error = distance_from_goal
    Actuator_Output = Kp * distance_from_goal + Ki * accumulative_error + Kd * change_in_error
    # A base motor speed to over come motor inertia
    if Actuator_Output < 0:
        Actuator_Output = Actuator_Output - motorMin
    else:
        Actuator_Output = Actuator_Output + motorMin
    # Clamp actuator output to motor range
    if Actuator_Output > outMax:
        Actuator_Output = outMax
    if Actuator_Output < outMin:
        Actuator_Output = outMin
    lastInput = input2
    lastTime = now
    return Actuator_Output

def on_button_pressed_a():
    global setPoint
    setPoint += 0 - 1
input.on_button_pressed(Button.A, on_button_pressed_a)

def motorController(speed: number):
    leftMotorBias = 0
    rightMotorBias = 0
    if speed > 0:
        pins.digital_write_pin(DigitalPin.P13, 1)
        pins.digital_write_pin(DigitalPin.P14, 0)
        pins.digital_write_pin(DigitalPin.P15, 1)
        pins.digital_write_pin(DigitalPin.P16, 0)
    if speed < 0:
        pins.digital_write_pin(DigitalPin.P13, 0)
        pins.digital_write_pin(DigitalPin.P14, 1)
        pins.digital_write_pin(DigitalPin.P15, 0)
        pins.digital_write_pin(DigitalPin.P16, 1)
    pins.analog_write_pin(AnalogPin.P0, abs(speed + rightMotorBias))
    pins.analog_set_period(AnalogPin.P0, 2500)
    pins.analog_write_pin(AnalogPin.P1, abs(speed + leftMotorBias))
    pins.analog_set_period(AnalogPin.P1, 2500)

def on_button_pressed_b():
    global setPoint
    setPoint += 1
input.on_button_pressed(Button.B, on_button_pressed_b)

def motorStop():
    pins.digital_write_pin(DigitalPin.P13, 0)
    pins.digital_write_pin(DigitalPin.P14, 0)
    pins.digital_write_pin(DigitalPin.P15, 0)
    pins.digital_write_pin(DigitalPin.P16, 0)
"""

Use to trim motors if one is stronger than the other

"""
"""

Motor state setup

"""
lastInput = 0
Actuator_Output = 0
last_error = 0
change_in_error = 0
accumulative_error = 0
distance_from_goal = 0
lastTime = 0
timeChange = 0
skipDebug = 0
speed = 0
outMin = 0
outMax = 0
Kd = 0
Ki = 0
Kp = 0
setPoint = 0
motorMin = 0
y: number = 0
# GY-521. Specific values calculated by separate calibration program
offSets = [2791, 593, 1065, 122, -29, 1]
# required to increase accuracy of readings
selfCal: bool = microbit_GY521.calibrate_Sensors(offSets)
if not (selfCal):
    basic.show_leds("""
        # . . # .
        # . # # #
        # . . # .
        # . # # #
        # . . # .
        """)
    basic.pause(30000)
# Manage baseline motor speed ie the value from which motor values have an affect
motorMin = 140
# PID state setup
setPoint = -1
# Now set dynamically on start up
# 102
Kp = 100
# 68
Ki = 76
# 60
Kd = 52
outMax = 1023
outMin = -1023
led.plot_bar_graph(1000, 1023)
basic.pause(500)
led.plot_bar_graph(750, 1023)
basic.pause(500)
led.plot_bar_graph(500, 1023)
basic.pause(500)
led.plot_bar_graph(250, 1023)
basic.pause(500)
basic.show_leds("""
    . # . . .
    # . . # .
    # . . . .
    # . . # .
    . # . . .
    """)
setPoint = microbit_GY521.compute_y()

def on_forever():
    global y, speed, skipDebug
    # keep to 200hz
    basic.pause(2)
    y = microbit_GY521.compute_y()
    # break;
    if y > 360:
        basic.show_leds("""
            # . . . .
            . # . # .
            . # . . .
            . # . # .
            # . . . .
            """)
    speed = PID(y)
    skipDebug += 1
    if y > -80 and y < 80:
        # This prevents error in the PID from accumulating when robot laying down
        motorController(speed)
    else:
        motorStop()
basic.forever(on_forever)
