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


def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,
        10, 0, 0,  # x, y, z velocity in m/s  (not used)
        # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

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
    # vehicle.simple_takeoff(5)  # Take off to target altitude

def actuator_test(motor_num):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,  # command
        0,  # confirmation
        motor_num,    # servo number
        1800,          # servo position between 1000 and 2000
        0, 0, 0, 0, 0)    # param 3 ~ 7 not used

    vehicle.send_mavlink(msg)
# Function to arm and then takeoff to a user specified altitude


def arm_and_takeoff(aTargetAltitude):

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

    # print("Taking off!")
    # vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude



# Initialize the takeoff sequence to 15m
# arm_and_takeoff(5)


print("Take off complete")

# Hover for 10 seconds
# time.sleep(15)
# goto_position_target_local_ned(-35.363,149.16, 5)
print("Servo test start")
actuator_pre_arm()
actuator_test(0)
actuator_test(1)
actuator_test(2)
actuator_test(3)
time.sleep(15)
print("Servo test end")
time.sleep(15)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

time.sleep(15)

# Close vehicle object
vehicle.close()
