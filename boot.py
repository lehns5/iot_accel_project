# SETUP WIFI CONNECTION
import network
from time import sleep

sta_if = network.WLAN(network.STA_IF)
networks = [['domih', 'DomisPW1'], ['Tripple L', '15263748']]
connected_network = None 

def connect_to_network(ssid, password):
    global connected_network
    sta_if.active(True)
    sta_if.disconnect()  # Vorherige Verbindungen abbrechen
    sta_if.connect(ssid, password)
    for _ in range(10):  # Versucht für 10 Sekunden eine Verbindung herzustellen
        if sta_if.isconnected():
            connected_network = (ssid, password)
            return True
        sleep(1)
    return False

if not sta_if.isconnected():
    print('Connecting to network...')
    for ssid, password in networks:
        if connect_to_network(ssid, password):
            break

if sta_if.isconnected():
    print('Network config:', sta_if.ifconfig())
else:
    print('Unable to connect to any network.')
