from __future__ import print_function
from dronekit import connect, VehicleMode
import time


#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

def actuator_pre_arm():
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(5)  # Take off to target altitude

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

actuator_pre_arm()
time.sleep(15)

@vehicle.on_attribute('ch1out')
def ch1out_listener(self, name, msg):
    print('%s attribute is %s' % (name,msg))

# Get all original channel values (before override)
print("Channel values from RC Tx:", vehicle.channels)

# Override channels
print("\nChannel overrides: %s" % vehicle.channels.overrides)

vehicle.channels.overrides['1'] = 2000
vehicle.channels.overrides['2'] = 800
vehicle.channels.overrides['3'] = 2000
vehicle.channels.overrides['4'] = 2000


print(" Channel overrides: %s" % vehicle.channels.overrides)

#Close vehicle object before exiting script
print("\nClose vehicle object")

vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")