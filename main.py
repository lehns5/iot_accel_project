import machine
import array
from imu import MPU6050
from machine import Pin, SoftI2C, unique_id
import time
from time import sleep, sleep_ms
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
led = Pin(13, Pin.OUT)
button = Pin(21, Pin.IN, machine.Pin.PULL_UP)

#mqtt setup
SERVER ='hairdresser.cloudmqtt.com'
CLIENT_ID= ubinascii.hexlify(unique_id())
PORT=15454 
TOPIC_Sensor=b'MPU6050/w3r/hahld1/UID'
TOPIC_CMD=b'CMD/Phone'
USERNAME = 'ebiswygf'
PASSWORD = 'NUgfXT68DID3'

#global variables
test = 0
alarm_level = 80
send = False
accel_value = []  #List to store acceleration magnitudes
filtered_data = [] # Accel Values Filtered with moving avg.
offset_xyz=[-0.04,0,0.24] #Offsets for Sensor calibration
state = 'record' #Initial State
exercise_initialized = False #Exercise Initialization Flag
repetitions = 0 #Repetitions
axis = 'z'
count = 1
reps = 0
increasing = False
first_repetition = True
max_value_first_repetition = 0
max_value_current_repetition = 0
current_max_accel = 0

# Parameters
PEAK_WINDOW = 10
MOVEMENT_THRESHOLD = 0.5
MOVEMENT_COUNT_THRESHOLD = 5
THRESHOLD = 0.3
WINDOW_SIZE = 5 # Number of Values that the mean is calculated over for filtered data
frequency = 20 # Frequency in Hz that is recorded
transmitt_density = 10 # How many points transmitted per second of recording
reduction_factor = (round(frequency/transmitt_density)) # factor for calculations

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
        time.sleep(1)

#functions

def reduce_data(data):
    data_red = []
    for i in range(math.floor(len(data)/reduction_factor)):
        data_red.append(data[i*reduction_factor])
    return data_red

def get_accel(axis,values):
    if axis == 'z':
        accel = (values-1)*9.81
    return accel

def send_data(Topic,data,max_accel,reps):
    json_data = {
    'data': data,
    'max_accel': max_accel,
    'reps': reps}
    payload = ujson.dumps(json_data)
    client.publish(Topic, payload)

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
    led(0)
    if axis == 'z':
        if len(accel_value) > WINDOW_SIZE:
            accel_value.pop(0)
            g = (mpu_data()[5]) + offset_xyz[2]
        else:
            g = (mpu_data()[5]) + offset_xyz[2]
        
        # Get magnitude and add to list
        accel = round(get_accel('z', g), 1)
        accel_value.append(((count * (1000 / frequency)) / 1000, accel))  # appends a tuple (time in second, acceleration value)
        if len(accel_value) >= WINDOW_SIZE:
            filtered_data.append((((count - math.floor(WINDOW_SIZE / 2)) * (1000 / frequency)) / 1000, moving_average(accel_value)))
        count += 1
        
        if len(filtered_data) > 1:
            detect_peaks_troughs(filtered_data[-1][1], filtered_data[-2][1])
        
        # Print current acceleration
        #if len(filtered_data) > 0:
            #print(accel_value[-1], filtered_data[-1])

        sleep_ms(int(1000 / frequency))


def standby():
    global accel, state, reps, max_value_current_repetition, first_repetition
    led(1)
    accel_value.clear()  # Clear the accel
    state = 'standby'

#subs
def subs(topic, msg):
    global state,count,alarm_level, first_repetition, reps
    data = ujson.loads(msg)
    if data["msg"] == "record":
        if state == "record":
            pass
        else:
            state = "record"
            alarm_level = data["alarm"]
            first_repetition = True
            reps = 0
            print(alarm_level)
            print(state)
    elif data["msg"] == "standby":
        if state == "standby":
            pass
        else:
            state = "standby"
            count == 0
            print('sent data',reduce_data(filtered_data))
            print(state)
            if send == True:
                send_data(TOPIC_Sensor,reduce_data(filtered_data),test,test)


def detect_peaks_troughs(current_value, previous_value):
    global reps, exercise_initialized, peak_detected, movement_count, max_accel_first_rep, current_max_accel
    global MOVEMENT_THRESHOLD, MOVEMENT_COUNT_THRESHOLD, THRESHOLD, alarm_level, state

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

            if movement_count == 2:
                if current_max_accel < THRESHOLD:
                    print("Max acceleration of current repetition below threshold, switching to standby")
                    print("Max acceleration last repetition: ", current_max_accel)
                    exercise_initialized = False
                    movement_count = 0
                    state = 'standby'
                    #Erst in Standby wenn Stopp geschickt wird, hier Meldung an node-rode dass Value erreicht.
                else:
                    reps += 1
                    print(current_max_accel)
                    print("Reps:", reps)
                    movement_count = 0  # Reset for next repetition
                    current_max_accel = 0

            # Update max acceleration values
            if movement_count == 1:
                max_accel_first_rep = max(max_accel_first_rep, current_max_accel)
                print("Max acceleration of first repetition:", max_accel_first_rep)
                THRESHOLD = (max_accel_first_rep * alarm_level) / 100
                print("Threshold set to:", THRESHOLD)


"""
    # Apply moving average filter
    filtered_accel = moving_average(accel_data, new_accel)
    # Detect peaks and troughs
    detect_peaks_troughs(filtered_accel, prev_value)
    # Update previous value
    prev_value = filtered_accel
    # Increment index for moving average buffer
    index += 1
    # Small delay to control the sampling rate
    time.sleep(0.01)"""

client=MQTTClient(CLIENT_ID,SERVER,PORT, USERNAME, PASSWORD)
client.set_callback(subs)
client.connect()
print("Connected to %s, subscribed to %s topic" % (SERVER, TOPIC_CMD))
client.subscribe(TOPIC_CMD)

_thread.start_new_thread(mqtt_thread, ())
#_thread.start_new_thread(reps, ())
_thread.start_new_thread(mpu_data, ())

while True:
    if state == 'record':
        record(axis)
    elif state == 'standby':
        standby()
