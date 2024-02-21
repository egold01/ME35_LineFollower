import RPi.GPIO as GPIO
import math
import time

# Assign GPIO pin numbers to variables
s2_1 = 16
s3_1 = 18
sig_1 = 22 #labeled "out" on your board
cycles = 10

s2_2 = 33
s3_2 = 35
sig_2 = 37


motor1a = 38
motor1b = 40

motor2a = 31
motor2b = 29


#TODO - get correct values
# Color Constants
#RED = 45000
# BLUE = 
# GREEN = 

# RED = (45000,0,0)
# GREEN = (0,45000,0)
# BLUE = (0,0,45000)
# PURPLE = (45000,0,45000)

# Color values for WHITE
RED = 45000
GREEN = 45000
BLUE = 45000

KP = 1.0
KI = 0.05
KD  = 1.0

MIN_DC = 30
MAX_DC = 40

# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)

# Color sensor 1 pins
GPIO.setup(s2_1, GPIO.OUT)
GPIO.setup(s3_1, GPIO.OUT)
GPIO.setup(sig_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#Color sensor 2 pins
GPIO.setup(s2_2, GPIO.OUT)
GPIO.setup(s3_2, GPIO.OUT)
GPIO.setup(sig_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Motor 1 Pins
GPIO.setup(motor1a, GPIO.OUT)
GPIO.setup(motor1b, GPIO.OUT)

# Motor 2 Pins
GPIO.setup(motor2a, GPIO.OUT)
GPIO.setup(motor2b, GPIO.OUT)

# Set up PWM pins and start motor 1
pwm_pin1 = GPIO.PWM(motor1b, 1000)  # channel=12 frequency=50Hz
pwm_pin1.start(MIN_DC) # Start with a 50% duty cycle

# Set up PWM pins and start motor 2
pwm_pin2 = GPIO.PWM(motor2a, 1000)  # channel=12 frequency=50Hz
pwm_pin2.start(MIN_DC) # Start with a 50% duty cycle

GPIO.output(motor1a, GPIO.LOW)
GPIO.output(motor2b, GPIO.LOW)

def DetectColors(s2, s3, sig):
    # Detect red values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration
    print("red value - ", red)

    # Detect blue values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    blue = cycles / duration
    print("blue value - ", blue)

    # Detect green values
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration
    print("green value - ", green)
    return (red, green, blue)

try:
    path_color = 'red' #'green', 'blue', 'purple'
    total_error_1 = 0
    total_error_2 = 0
    prev_error_1 = 0
    prev_error_2 = 0
    sum_err_1 = 0
    sum_err_2 = 0
    duty_cycle_1 = MIN_DC
    duty_cycle_2 = MIN_DC

    red_error_1 = 0
    red_error_2 = 0
    green_error_1 = 0
    green_error_2 = 0
    blue_error_1 = 0
    blue_error_2 = 0
    while True:
        # Get color reading and calculate error
        colors_1 = DetectColors(s2_1, s3_1, sig_1)
        colors_2 = DetectColors(s2_2, s3_2, sig_2)

        # Get individual color error values
        # Color error values cannot be negative
        if colors_1[0] > RED:
            red_error_1 = 0
        else:
            red_error_1 = RED - colors_1[0]
        
        if colors_1[1] > GREEN:
            green_error_1 = 0
        else:
            green_error_1 = GREEN - colors_1[1]

        if colors_1[2] > BLUE:
            blue_error_1 = 0
        else:
            blue_error_1 = BLUE - colors_1[2]

        if colors_2[0] > RED:
            red_error_2 = 0
        else:
            red_error_2 = RED - colors_2[0]
        
        if colors_2[1] > GREEN:
            green_error_2 = 0
        else:
            green_error_2 = GREEN - colors_2[1]

        if colors_2[2] > BLUE:
            blue_error_2 = 0
        else:
            blue_error_2 = BLUE - colors_2[2]
        # if path_color == 'red':
        #     red_error = RED(0) - colors(0)
        #     green_error = RED(1) - colors(1)
        #     blue_error = RED(2) - colors(2)
        # if path_color == 'green':
        #     red_error = GREEN(0) - colors(0)
        #     green_error = GREEN(1) - colors(1)
        #     blue_error = GREEN(2) - colors(2)
        # if path_color == 'blue':
        #     red_error = BLUE(0) - colors(0)
        #     green_error = BLUE(1) - colors(1)
        #     blue_error = BLUE(2) - colors(2)
        # if path_color == 'purple':
        #     red_error = PURPLE(0) - colors(0)
        #     green_error = PURPLE(1) - colors(1)
        #     blue_error = PURPLE(2) - colors(2)
        
        total_error_1 = math.sqrt(red_error_1**2 + green_error_1**2 + blue_error_1**2)
        total_error_2 = math.sqrt(red_error_2**2 + green_error_2**2 + blue_error_2**2)

        # Calculate PID 1
        p_1 = total_error_1 * KP
        sum_err_1 =+ total_error_1
        i_1 = sum_err_1 * KI
        d_1 = (total_error_1 - prev_error_1) * KD
        prev_error_1 = total_error_1
        sum_pid_1 = p_1 #+ i_1 + d_1
        
        # Calculate PID
        p_2 = total_error_2 * KP
        sum_err_2 =+ total_error_2
        i_2 = sum_err_2 * KI
        d_2 = (total_error_2 - prev_error_2) * KD
        prev_error_2 = total_error_2
        sum_pid_2 = p_2 #+ i_2 + d_2
        # if path_color == 'red':
        #     min_error = 0
        #     max_error = KP * math.sqrt(RED(0)**2 + RED(1)**2 + RED(2)**2)
        #     duty_cycle = sum_pid / (RED(0) * KP) * 100
        #     duty_cycle = (sum_pid/(max_error - min_error)) * (100 - MIN_DC) + MIN_DC

        # Convert PID to PWM
        min_error = 0
        max_error = KP * math.sqrt(RED**2 + GREEN**2 + BLUE**2)
        duty_cycle_1 = (sum_pid_1/(max_error - min_error)) * (MAX_DC - MIN_DC) + MIN_DC
        duty_cycle_2 = (sum_pid_2/(max_error - min_error)) * (MAX_DC - MIN_DC) + MIN_DC
        # Update the duty cycle
        pwm_pin1.ChangeDutyCycle(duty_cycle_1)
        pwm_pin2.ChangeDutyCycle(duty_cycle_2) 
    
        # Convert sum_pid to duty cycle
        '''min_dc = .5
        max_dc = 1

        min_error = 0
        max_error = KP * max_color_val

        error = KP * (max_color_val - curr_color_val)

        (error / (max_error - min_error)) * (max_dc - min_dc) + min_dc'''



except KeyboardInterrupt:
    GPIO.cleanup()