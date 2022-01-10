from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14551')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

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

    print("Motors armed!")
    # vehicle.simple_takeoff(5)  # Take off to target altitude

def actuator_test(motor_num, set_reset):
    # get servo function - what this motor does
    print("GOT PARAM: ", vehicle.parameters[f'SERVO{motor_num}_FUNCTION'])
    time.sleep(1)

    # set servo function - change to 1 for RCPassThru
    vehicle.parameters[f'SERVO{motor_num}_FUNCTION'] = set_reset
    time.sleep(1)

    # send PWM to motor - if you want to test something manually
    # or don't send anything - drone should start falling off

# Function to arm and then takeoff to a user specified altitude

actuator_pre_arm()

actuator_test(1, 33)

arm_and_takeoff(10)

print("Start goto NED")
time.sleep(5)
print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(5)
# actuator test - DO_SET_SERVO should fail
# Check cmd where sim is run -> ACK for DO_SET_SERVO reads failed.
actuator_test(1, 1)

time.sleep(10)

print("End goto NED")

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

time.sleep(5)

# Close vehicle object
vehicle.close()
