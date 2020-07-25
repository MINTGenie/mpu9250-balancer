// increase to reduce forward power to the left
function read_gyro_angle_rate () {
    InvMPU.read_gyro()
    return 0 - InvMPU.gyro_y
}
function control_loop () {
    // degrees * 100 from vertical
    est_angle = read_accel_tilt_angle()
    last_time = input.runningTime()
    while (true) {
        current_time = input.runningTime()
        delta_t = current_time - last_time
        last_time = current_time
        est_angle = updateAngle(est_angle, delta_t)
        err = est_angle - TARGET_ANGLE
        if (motor_on && (err > 3000 || err < -3000)) {
            last_err = 0
            i_err = 0
            motor_coast()
            motor_on = false
        }
        if (motor_on) {
            d_err = (err - last_err) * 1000 / delta_t
            last_err = err
            i_err = i_err + err * delta_t
            u = err * KP + d_err * KD + i_err * KI
            motor_out = u / 3000
            motor_right = motor_out - motor_bias
            motor_left = motor_out + motor_bias
            serial.writeNumber(err)
            serial.writeLine("")
            // serial.writeNumber(motor_right)
            motor_move(pins.map(
            motor_left,
            0 - MOTOR,
            MOTOR,
            -100,
            100
            ), pins.map(
            motor_right,
            0 - MOTOR,
            MOTOR,
            -100,
            100
            ))
        } else if (err <= 500 && err >= -500) {
            motor_on = true
        }
        basic.pause(5)
    }
}
function read_accel_tilt_angle () {
    InvMPU.read_accel()
    // degrees * 100
    return Trig.atan2(0 - InvMPU.accel_z, InvMPU.accel_x)
}
function setup () {
    basic.showIcon(IconNames.Happy)
    while (true) {
        while (!(input.buttonIsPressed(Button.A))) {
            basic.pause(10)
        }
        if (InvMPU.find_mpu()) {
            break;
        }
        basic.showIcon(IconNames.No)
    }
    InvMPU.reset_mpu(0, 0)
    basic.pause(100)
    basic.clearScreen()
    // set this for your sensor
    InvMPU.set_gyro_bias(0, 0, 0)
}
function updateAngle (est_angle: number, delta_t_ms: number) {
    // degrees * 100
    accel_angle = read_accel_tilt_angle()
    // (degrees * 100) per second
    gyro_angle_rate = read_gyro_angle_rate() * 200000 / 32768
    // degrees * 100
    gyro_angle_change = gyro_angle_rate * delta_t_ms / 1000
    new_est_angle = (49 * (est_angle + gyro_angle_change) + accel_angle) / 50
    return new_est_angle
}
function motor_coast () {
    robotbit.MotorStop(robotbit.Motors.M1A)
    robotbit.MotorStop(robotbit.Motors.M2A)
}
function motor_move (left: number, right: number) {
    if (left >= 0) {
        robotbit.MotorRun(robotbit.Motors.M1A, 0 - Math.abs(left))
    } else {
        robotbit.MotorRun(robotbit.Motors.M1A, Math.abs(left))
    }
    if (right >= 0) {
        robotbit.MotorRun(robotbit.Motors.M2A, 0 - Math.abs(right))
    } else {
        robotbit.MotorRun(robotbit.Motors.M2A, Math.abs(right))
    }
}
let new_est_angle = 0
let gyro_angle_change = 0
let gyro_angle_rate = 0
let accel_angle = 0
let motor_left = 0
let motor_right = 0
let motor_out = 0
let u = 0
let d_err = 0
let i_err = 0
let last_err = 0
let motor_on = false
let err = 0
let delta_t = 0
let current_time = 0
let last_time = 0
let est_angle = 0
let motor_bias = 0
let KD = 0
let KI = 0
let KP = 0
let MOTOR = 0
let TARGET_ANGLE = 0
// Tuning Parameters
// forward tilt from vertical in degrees * 100
TARGET_ANGLE = -8600
// less = strong
MOTOR = 100
// Realist - Correct only current error
KP = 400
// Pessimist - Correct accumulated error
KI = 3
// Flip-flopper - Correct difference between current & last error
KD = 8
// increase to reduce forward power to the left
motor_bias = -5
serial.redirectToUSB()
setup()
control_loop()
