from pi_gpio_control.gpio_control import Gpio_Control

pin_mode = "BOARD"
pin_type_motor = "OUT"
pin_type_sensor = "IN"

# Motor control:
motor_dir = Gpio_Control(pin_mode, pin_type_motor, [16])
motor_pwm = Gpio_Control(pin_mode, pin_type_motor, [12])

# Sensor
sensor = Gpio_Control(pin_mode, pin_type_sensor, [37])

continue_code = True

while continue_code == True:

    motor_dir.state_on()
    value = sensor.read_pin()

    while value[0] == 0:

        value = sensor.read_pin()
        motor_pwm.set_pwm(50, 20, False)
        print(value)

    motor_pwm.set_pwm(50, 20, True)

