# SETUP WIFI CONECTION
import network
from time import sleep

sta_if = network.WLAN(network.STA_IF)
networks=[['domih','DomisPW1'],['Tripple L', '15263748']]
SSID = 'domih'
PW = 'DomisPW1'

if not sta_if.isconnected():
  print('connecting to network...')
  for i in range(len(networks)):
    try:
      sta_if.active(True)
      sta_if.connect(networks[i][0], networks[i][1])
    except:
      pass

  while not sta_if.isconnected():
    pass

print('network config:', sta_if.ifconfig())
