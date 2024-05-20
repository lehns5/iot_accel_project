# SETUP WIFI CONECTION
import network
from time import sleep
from WIFI_CONFIG import networks

connected_network = None

def connect_to_wifi():
    global connected_network
    sta_if = network.WLAN(network.STA_IF)
    
    if not sta_if.isconnected():
        print('Connecting to network...')
        for ssid, password in networks:
            sta_if.active(True)
            sta_if.connect(ssid, password)
            for _ in range(10):
                if sta_if.isconnected():
                    break
                sleep(1)
            if sta_if.isconnected():
                print(f'Connected to {ssid}')
                connected_network = (ssid, password)
                break
        else:
            print('Failed to connect to any network')
    
    if sta_if.isconnected():
        print('Network config:', sta_if.ifconfig())

connect_to_wifi()
