import RPi.GPIO as GPIO
import time

# Assign GPIO pin numbers to variables
s2 = 16
s3 = 18
sig = 22 #labeled "out" on your board
cycles = 10

# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def DetectColor():
    # Detect red values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration
    #print("red value - ", red)

    # Detect blue values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    blue = cycles / duration
    #print("blue value - ", blue)

    # Detect green values
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration
    #print("green value - ", green)
    return(red,green,blue)

reds = []
greens = []
blues = []
try:
    for i in range(1000):

        red,green,blue = DetectColor()
        reds.append(red)
        greens.append(greens)
        blues.append(blue)
        print(i)

    print("reds")
    avg_red = sum(reds)/int(len(reds))
    min_red = min(reds)
    max_red = max(reds)

    print("greens")
    avg_green = sum(greens)/int(len(greens))
    min_green = min(greens)
    max_green = max(greens)

    print("blues")
    avg_blue = sum(blues)/int(len(blues))
    min_blue = min(blues)
    max_blue = max(blues)

    print("datas")
    print("averages",avg_red, avg_green, avg_blue)
    print("maxs",max_red,max_green,max_blue)
    print("mins",min_red, min_green, min_blue)
    GPIO.cleanup()


except KeyboardInterrupt:
    GPIO.cleanup()
