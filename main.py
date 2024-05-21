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
import boot
from ota import OTAUpdater

#OTA
SSID, PASSWORD = boot.connected_network
firmware_url = "https://raw.githubusercontent.com/lehns5/iot_accel_project/main"
ota_updater = OTAUpdater(SSID, PASSWORD, firmware_url, "main.py")
ota_updater.download_and_install_update_if_available()

#Sensor Setup
i2c = SoftI2C(sda=Pin(23), scl=Pin(22), freq=400000)
mpu = MPU6050(i2c)

#Pin setups
led = Pin(13, Pin.OUT)

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
alarm_level = 80 #acceleration alarm level
send = True #Determines if data is sent to broker or not
accel_value = []  #List to store acceleration magnitudes
filtered_data = [] # Accel Values Filtered with moving avg.
offset_xyz=[-0.04,0,0.17] #Offsets for Sensor calibration
state = 'standby' #Initial State
exercise_initialized = False #Exercise Initialization Flag
repetitions = 0 #Repetitions
axis = 'z'#determines if z axis or xyz axis are used
count = 1 #global
reps = 0 #numver of reps recorded
increasing = False
first_repetition = True
max_value_first_repetition = 0
max_value_current_repetition = 0
current_max_accel = 0
max_accel = 7.0 # Test variable, will be changed
avg_accel = 6.0 # Test variable, will be changed

# Parameters
PEAK_WINDOW = 10
MOVEMENT_THRESHOLD = 0.5
MOVEMENT_COUNT_THRESHOLD = 5
THRESHOLD = 0.3
WINDOW_SIZE = 5 # Number of Values that the mean is calculated over for filtered data
frequency = 100 # Frequency in Hz that is recorded
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

def calibrate_sensor(axis):
    if axis == 'z':
        num_measurements = 10
        value = 0
        for i in range(num_measurements):
            value+=mpu.accel.z
    return round(1-(value/num_measurements),2)

def send_data(Topic,data,max,avg,reps):
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
    global state,count,alarm_level, first_repetition, reps, offset_xyz
    data = ujson.loads(msg)
    if data["msg"] == "record":
        if state == "record":
            pass
        else:
            if "alarm" in data:
                alarm_level = int(data["alarm"])
            first_repetition = True
            reps = 0
            state = "record"
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
                send_data(TOPIC_Sensor,reduce_data(filtered_data),max_accel,avg_accel,reps)
    elif data["msg"] == "calibrate":
        if axis == 'z':
            offset_xyz[2] = calibrate_sensor('z')
            print('z-offset set to:', offset_xyz[2])

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
                    print("Best progress reached!")
                    print("Max acceleration last repetition: ", current_max_accel)
                    reps += 1
                    exercise_initialized = False
                    movement_count = 0
                    send_mqtt(TOPIC_Alarm, "on")
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

client=MQTTClient(CLIENT_ID,SERVER,PORT, USERNAME, PASSWORD)
client.set_callback(subs)
client.connect()
print("Connected to %s, subscribed to %s topic" % (SERVER, TOPIC_CMD))
client.subscribe(TOPIC_CMD)

_thread.start_new_thread(mqtt_thread, ())
_thread.start_new_thread(mpu_data, ())

while True:
    if state == 'record':
        record(axis)
    elif state == 'standby':
        standby()
