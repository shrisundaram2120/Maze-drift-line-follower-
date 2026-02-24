from machine import Pin, I2C, PWM
import time

# -----------------------
# Motor Pins + PWM
# -----------------------
in1 = Pin(3, Pin.OUT)
in2 = Pin(4, Pin.OUT)
in3 = Pin(6, Pin.OUT)
in4 = Pin(7, Pin.OUT)

# PWM speed (0-65535 for MicroPython)
speed_pwm = PWM(Pin(15))
speed_pwm.freq(1000)
speed_pwm.duty_u16(40000)  # base speed

# -----------------------
# IR Sensors
# -----------------------
sensors = [Pin(pin, Pin.IN) for pin in [26,27,28,29,10,11,12,13]]

def read_line():
    # returns list of 0=black,1=white
    return [s.value() for s in sensors]

# -----------------------
# EEPROM (I2C)
# -----------------------
i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
EEPROM_ADDR = 0x50  # 24C32

def write_eeprom(addr, data):
    i2c.writeto_mem(EEPROM_ADDR, addr, data)

def read_eeprom(addr, nbytes):
    return i2c.readfrom_mem(EEPROM_ADDR, addr, nbytes)

# -----------------------
# Motor Control
# -----------------------
def forward(left_pwm=40000, right_pwm=40000):
    in1.value(1)
    in2.value(0)
    in3.value(1)
    in4.value(0)
    # adjust PWM if needed

def stop():
    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(0)

def turn_left():
    in1.value(0)
    in2.value(1)
    in3.value(1)
    in4.value(0)
    time.sleep(0.3)
    stop()

def turn_right():
    in1.value(1)
    in2.value(0)
    in3.value(0)
    in4.value(1)
    time.sleep(0.3)
    stop()

def rotate_360():
    in1.value(1)
    in2.value(0)
    in3.value(0)
    in4.value(1)
    time.sleep(1.5)
    stop()

# -----------------------
# PID Line Following
# -----------------------
Kp = 25   # tune these values
Ki = 0
Kd = 15

last_error = 0
integral = 0

def pid_control(line):
    global last_error, integral
    # calculate position
    weights = [-3,-2,-1,0,0,1,2,3]  # adjust for sensor layout
    position = 0
    count = 0
    for i,val in enumerate(line):
        if val==0:  # black
            position += weights[i]
            count += 1
    if count==0:
        error = 0  # all white
    else:
        error = position / count
    
    integral += error
    derivative = error - last_error
    output = Kp*error + Ki*integral + Kd*derivative
    last_error = error

    # adjust motor speed
    base_speed = 40000
    left_speed = max(0, min(65535, int(base_speed - output)))
    right_speed = max(0, min(65535, int(base_speed + output)))

    forward(left_speed, right_speed)

# -----------------------
# Dry Run: Explore Maze
# -----------------------
path = []  # store moves during dry run

def dry_run():
    rotate_360()
    while True:
        line = read_line()
        pid_control(line)

        # detect junctions / black boxes
        # placeholder logic:
        if sum(line)==0:  # all black -> junction
            # decide turn, append to path
            path.append('L')  # example
            turn_left()

        elif sum(line)==8:  # all white -> dead end
            stop()
            break

        time.sleep(0.05)

    # save path to EEPROM
    for i, move in enumerate(path):
        write_eeprom(i, move.encode())

# -----------------------
# Actual Run: Follow Shortest Path
# -----------------------
def actual_run():
    n = len(path)
    stored = read_eeprom(0, n)
    for b in stored:
        move = chr(b)
        if move=='S':
            forward()
        elif move=='L':
            turn_left()
        elif move=='R':
            turn_right()
        time.sleep(0.2)
        stop()

# -----------------------
# Main
# -----------------------
def main():
    dry_run()      # explore and store path
    actual_run()   # follow shortest path

main()