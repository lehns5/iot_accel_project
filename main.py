import machine
from imu import MPU6050
from machine import Pin, SoftI2C, unique_id
import time
from time import sleep_ms
import math
from umqttsimple import MQTTClient 
import ujson
import ubinascii
import _thread
import vector3d



#Sensor Setup
i2c = SoftI2C(sda=Pin(23), scl=Pin(22), freq=400000)
mpu = MPU6050(i2c)

#Pin setups
ob_led = Pin(13, Pin.OUT)
green_led = Pin(17, Pin.OUT)
yellow_led = Pin(16, Pin.OUT)

#mqtt setup
SERVER ='hairdresser.cloudmqtt.com'
CLIENT_ID= ubinascii.hexlify(unique_id())
PORT=15454 
TOPIC_Sensor=b'MPU6050/w3r/hahld1/UID'
TOPIC_CMD=b'CMD/Phone'
TOPIC_Alarm=b'MPU6050/w3r/hahld1/ACCEL_ALARM'
USERNAME = 'ebiswygf'
PASSWORD = 'NUgfXT68DID3'

#global variables
setup_time = 5 #setup time before record starts
alarm_level = 80 #acceleration alarm level
send = True #Determines if data is sent to broker or not
accel_value = []  #List to store acceleration magnitudes
filtered_data = [] # Accel Values Filtered with moving avg.
offset_xyz=[-0.04,0,0.17] #Offsets for Sensor calibration
state = 'standby' #Initial State
exercise_initialized = False #Exercise Initialization Flag
axis = 'z'#determines if z axis or xyz axis are used
count = 1 #global
reps = 0 #number of reps recorded
first_repetition = True
current_max_accel = 0
max_accel = 0 #Max accel of the first 3 reps sending to node-red
avg_accel = 0 #Avg accel for sending to node-red
firs_three_reps = [] #List to safe the max accel of first 3 reps
current_max_accel_list = [] #List to safe the max accel of every rep
target_reps = 12 #Default value
target_stat = "both" #Default Value

#Parameters that can be changed for deeper specification of the training
MOVEMENT_THRESHOLD = 1.5 #Acceleration threashold to detect a movement
MOVEMENT_COUNT_THRESHOLD = 10 #Number of values that need in the movement threashold to detect a turning point
THRESHOLD = 0.3
WINDOW_SIZE = 5 # Number of Values that the mean is calculated over for filtered data
FREQUENCY = 50 # Frequency in Hz that is recorded

def mpu_data():
    gx = mpu.gyro.x
    gy = mpu.gyro.y
    gz = mpu.gyro.z
    ax = mpu.accel.x
    ay = mpu.accel.y
    az = mpu.accel.z
    return gx,gy,gz,ax,ay,az

#threads
def mqtt_thread():
    while True:
        client.check_msg()
        time.sleep_ms(100)

#functions
def led_blink():
    while True:
        if state == 'record':
            ob_led(0)
            yellow_led(0)
            green_led(1)
            time.sleep_ms(200)
            green_led(0)
            time.sleep_ms(800)
        elif state == 'standby':
            ob_led(1)
            green_led(0)
            yellow_led(0)
        elif state == 'calibrate':
            ob_led(1)
            green_led(1)
            yellow_led(1)
        elif state == 'setup':
            ob_led(0)
            green_led(0)
            yellow_led(1)

def get_accel(axis,values):
    if axis == 'z':
        accel = (values-1)*9.81
    return accel

def calibrate_sensor(axis):
    if axis == 'z':
        num_measurements = 20
        value = 0
        for i in range(num_measurements):
            value+=mpu.accel.z
    return round(1-(value/num_measurements),2)

def send_data(Topic,max,avg,reps):
    json_data = {
    'max_accel': max,
    'avg_accel': avg,
    'reps': reps}
    payload = ujson.dumps(json_data)
    client.publish(Topic, payload)

def send_mqtt(topic, message):
    client.publish(topic, message)

def moving_average(accel_value):
    if len(accel_value)-WINDOW_SIZE >0:
        sum=0
        for i in range(len(accel_value)-WINDOW_SIZE,len(accel_value)):
            sum+=(accel_value[i][1])
        return round(sum/WINDOW_SIZE,1)
    else:
        return accel_value[-1][0]

def record(axis):
    global count, accel_value
    if axis == 'z':
        if len(accel_value) > WINDOW_SIZE:
            accel_value.pop(0)
            g = (mpu_data()[5]) + offset_xyz[2]
        else:
            g = (mpu_data()[5]) + offset_xyz[2]
        
        # Get magnitude and add to list
        accel = round(get_accel('z', g), 1)
        accel_value.append(((count * (1000 / FREQUENCY)) / 1000, accel))  # appends a tuple (time in second, acceleration value)
        if len(accel_value) >= WINDOW_SIZE:
            filtered_data.append((((count - math.floor(WINDOW_SIZE / 2)) * (1000 / FREQUENCY)) / 1000, moving_average(accel_value)))
        if len(filtered_data) > MOVEMENT_COUNT_THRESHOLD:
            filtered_data.pop(0)

        count += 1
        
        if len(filtered_data) > 1:
            detect_peaks_troughs(filtered_data[-1][1])

        sleep_ms(int(1000 / FREQUENCY))


def standby():
    global accel_value, state, reps, first_repetition, filtered_data
    accel_value.clear()  # Clear the accel
    filtered_data.clear()
    state = 'standby'

#subs
def subs(topic, msg):
    global setup_time,state,count,alarm_level, first_repetition, reps, offset_xyz, current_max_accel_list, firs_three_reps, target_reps, target_stat
    data = ujson.loads(msg)
    if data["msg"] == "record":
        setup_time = int(data["setup"])
        state = 'setup'
        sleep_ms(setup_time*1000)
        if state == "record":
            pass
        else:
            if data["alarm"] != "0" and data["reps"] == "0":
                alarm_level = int(data["alarm"])
                target_stat = "alarm"
            elif data["reps"] != "0" and data["alarm"] == "0":
                target_reps = int(data["reps"])
                target_stat = "reps"
            elif (data["reps"] != "0") and (data["alarm"] != "0"):
                alarm_level = int(data["alarm"])
                target_reps = int(data["reps"])
                target_stat = "both"
            elif(data["reps"] == "0") and (data["alarm"] == "0"):
                target_reps = 12
                alarm_level = 80
                target_stat = "both"
            first_repetition = True
            reps = 0
            firs_three_reps = []
            current_max_accel_list = []
            state = "record"
            print(target_stat)
            print(alarm_level)
            print(target_reps)
            print(state)
    elif data["msg"] == "standby":
        if state == "standby":
            pass
        else:
            state = "standby"
            count == 0
            print(state)
            if send == True:
                send_data(TOPIC_Sensor,round(max_accel,2),round(avg_accel,2),reps)
    elif data["msg"] == "calibrate":
        state = 'calibrate'
        if axis == 'z':
            offset_xyz[2] = calibrate_sensor('z')
            print('z-offset set to:', offset_xyz[2])
            send_mqtt(TOPIC_Alarm, "calibrated")
            state = 'standby'

def detect_peaks_troughs(current_value):
    global reps, exercise_initialized, peak_detected, movement_count, max_accel_first_rep, current_max_accel
    global MOVEMENT_THRESHOLD, MOVEMENT_COUNT_THRESHOLD, THRESHOLD, alarm_level, max_accel, avg_accel
    global firs_three_reps
    global current_max_accel_list

    check_accel = 0
    # Check if movement has started
    if not exercise_initialized and abs(current_value) > MOVEMENT_THRESHOLD and len(filtered_data) >= MOVEMENT_COUNT_THRESHOLD:
        exercise_initialized = True
        peak_detected = False
        movement_count = 0
        max_accel_first_rep = 0  # Reset max_accel_first_rep for the first repetition
        current_max_accel = 0  # Reset current_max_accel for the current repetition
        print("First movement detected")

    if exercise_initialized:
        # Update the maximum acceleration for the current repetition if not in the rest phase
        if movement_count != 1:  # Ignore the relaxation phase
            current_max_accel = max(current_max_accel, abs(current_value))

        # Detect the beginning of a peak
        if any(abs(value[1]) > MOVEMENT_THRESHOLD for value in filtered_data[-MOVEMENT_COUNT_THRESHOLD:]) and not peak_detected:
            peak_detected = True
            print("Peak detected")

        # Detect the end of a peak
        if all(abs(value[1]) < MOVEMENT_THRESHOLD for value in filtered_data[-MOVEMENT_COUNT_THRESHOLD:]) and peak_detected:
            peak_detected = False
            movement_count += 1
            print("End of peak detected")
            print("Movement count:", movement_count)

            if reps < 3 and movement_count == 1:
                firs_three_reps.append(current_max_accel)
                print(firs_three_reps)
                if len(firs_three_reps) == 3:
                    max_accel = sum(firs_three_reps) / len(firs_three_reps)
                    print("Max acceleration first three reps: ", max_accel)
                    THRESHOLD = round((max_accel * alarm_level) / 100, 1)
                    print("Threshold set to:", THRESHOLD)

            if movement_count == 2:
                current_max_accel_list.append(current_max_accel)
                check_accel = round(current_max_accel, 1)
                print(current_max_accel_list)
                avg_accel = sum(current_max_accel_list) / len(current_max_accel_list)
                if ((check_accel <= THRESHOLD) and (reps > 2) and (target_stat == "alarm")):
                    reps += 1
                    exercise_initialized = False
                    movement_count = 0
                    print("Best progress reached!")
                    print(avg_accel)
                    print(reps)
                    send_mqtt(TOPIC_Alarm, "on")
                elif (reps == (target_reps-1)) and (target_stat == "reps"):
                    reps += 1
                    exercise_initialized = False
                    movement_count = 0
                    print("Best progress reached!")
                    print(avg_accel)
                    print(reps)
                    send_mqtt(TOPIC_Alarm, "on")
                elif (((check_accel <= THRESHOLD) and (reps > 2)) or (reps == (target_reps-1))) and (target_stat == "both"):
                    reps += 1
                    exercise_initialized = False
                    movement_count = 0
                    print("Best progress reached!")
                    print(avg_accel)
                    print(reps)
                    send_mqtt(TOPIC_Alarm, "on")
                else:
                    reps += 1
                    print("Reps:", reps)
                    movement_count = 0  # Reset for next repetition
                    current_max_accel = 0


client=MQTTClient(CLIENT_ID,SERVER,PORT, USERNAME, PASSWORD)
client.set_callback(subs)
client.connect()
print("Connected to %s, subscribed to %s topic" % (SERVER, TOPIC_CMD))
client.subscribe(TOPIC_CMD)

_thread.start_new_thread(mqtt_thread, ())
_thread.start_new_thread(led_blink, ())

while True:
    if state == 'record':
        record(axis)
    elif state == 'standby':
        standby()
