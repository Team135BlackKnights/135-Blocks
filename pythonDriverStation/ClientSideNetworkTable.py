import sys
import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize(server="10.1.35.2")

sd = NetworkTables.getTable("SmartDashboard")

i = 0
while True:
    print("handlerReturn", sd.getString("DataHandler", "not initialized"))

    sd.putNumber("dsTime", i)
    time.sleep(1)
    i += 1
ip = sys.argv[1]

NetworkTables.initialize(server="10.01.35.11")

sd = NetworkTables.getTable("SmartDashboard")

i = 0
while True:
    print("robotTime:", sd.getNumber("robotTime", -1))

    sd.putNumber("dsTime", i)
    time.sleep(1)
    i += 1