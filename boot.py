# SETUP WIFI CONNECTION
import network
from time import sleep
from ota import OTAUpdater

sta_if = network.WLAN(network.STA_IF)
SSID = "" #Insert your connection data here
PASSWORD = "" #Insert your connectiion data here
connected_network = None 


#OTA
firmware_url = "https://raw.githubusercontent.com/lehns5/iot_accel_project/"
ota_updater = OTAUpdater(SSID, PASSWORD, firmware_url, "main.py")
ota_updater.download_and_install_update_if_available()

#Network connection
def connect_to_network(ssid, password):
    global connected_network
    sta_if.active(True)
    sta_if.disconnect()  #Disconnect previous connection
    sta_if.connect(ssid, password)

if not sta_if.isconnected():
    print('Connecting to network...')
    connect_to_network(SSID, PASSWORD)

if sta_if.isconnected():
    print('Network config:', sta_if.ifconfig())
else:
    print('Unable to connect to any network.')
