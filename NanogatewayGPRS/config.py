""" LoPy LoRaWAN Nano Gateway configuration options """

import machine
import ubinascii

WIFI_MAC = ubinascii.hexlify(machine.unique_id()).upper()
# Set  the Gateway ID to be the first 3 bytes of MAC address + 'FFFE' + last 3 bytes of MAC address
GATEWAY_ID = WIFI_MAC[:6] + "FFFE" + WIFI_MAC[6:12]

SIM = 'internet.claro.com.ec'
SIM_USER = ''
SIM_PASS = ''

#SERVER = 'router.eu.thethings.network' #52.169.76.203
#SERVER = '52.169.76.203'
#PORT = 1700
SERVER = '190.15.132.17'
PORT = 1680
#SERVER = '192.168.0.101'
#PORT = 1680

NTP = "pool.ntp.org"
#NTP = "inocar.ntp.ec"
NTP_REGION = 0
NTP_PERIOD_S = 3600

# for EU868
#LORA_FREQUENCY = 868100000
#LORA_GW_DR = "SF7BW125" # DR_5
#LORA_NODE_DR = 5

# for US915
LORA_FREQUENCY = 903900000
LORA_GW_DR = "SF7BW125" # DR_3
LORA_NODE_DR = 3
