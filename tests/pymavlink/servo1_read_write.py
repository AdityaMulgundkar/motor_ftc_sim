"""
Example code of how to read and write vehicle parameters using pymavlink
"""

import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
# master = mavutil.mavlink_connection('com3')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n + 8,  # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )


# Request parameter
master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'SERVO1_FUNCTION',
    -1
)

# Print old parameter value
message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' %
      (message['param_id'], message['param_value']))

time.sleep(1)

# Set parameter value
# Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot.
# Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM.
# The parameter variable type is described by MAV_PARAM_TYPE in http://mavlink.org/messages/common.
master.mav.param_set_send(
    master.target_system, master.target_component,
    b'SERVO1_FUNCTION',
    1,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

# Read ACK
# IMPORTANT: The receiving component should acknowledge the new parameter value by sending a
# param_value message to all communication partners.
# This will also ensure that multiple GCS all have an up-to-date list of all parameters.
# If the sending GCS did not receive a PARAM_VALUE message within its timeout time,
# it should re-send the PARAM_SET message.
message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' %
      (message['param_id'], message['param_value']))

time.sleep(1)

# Request parameter value to confirm
master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'SERVO1_FUNCTION',
    -1
)

# Print new value in RAM
message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' %
      (message['param_id'], message['param_value']))


# command servo_1 to go from min to max in steps of 50us, over 2 seconds
for us in range(1100, 1900, 50):
    print("for us: %d start.", us)
    set_servo_pwm(0, us)
    time.sleep(0.125)
    # Print new parameter value
    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    print('name: %s\tvalue: %d' %
        (message['param_id'], message['param_value']))
    time.sleep(0.125)
    print("for us: %d end.", us)