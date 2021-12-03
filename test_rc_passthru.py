#!/usr/bin/env python3

import time
# Import mavutil
from pymavlink import mavutil
import enum
 
class ServoStatus(enum.Enum):
    disabled = 0
    enabled = 1

class ServoModes(enum.Enum):
    disabled = 0
    RCPassThru = 1
 
# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
# master = mavutil.mavlink_connection('com3')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

class MyServo:
    def __init__(self, id: int, status: ServoStatus):
        self.id = id
        self.status = status

def set_servo_function(servo: MyServo, mode: str):
    master.mav.param_set_send(
    master.target_system, master.target_component,
    f'SERVO{servo.id}_FUNCTION'.encode(),
    1,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    print('name: %s\tvalue: %d' %
        (message['param_id'], message['param_value']))

    time.sleep(1)

def set_servo_pwm(servo: MyServo, pwm: int):
    if(servo.status==ServoStatus.disabled):
        pwm = 0
    master.set_servo(channel=servo.id, pwm=1800)

if __name__ == '__main__':
    s1 = MyServo(id=1, status= ServoStatus.enabled)
    s2 = MyServo(id=2, status= ServoStatus.disabled)
    s3 = MyServo(id=3, status= ServoStatus.enabled)
    s4 = MyServo(id=4, status= ServoStatus.enabled)

    print("Setting servo modes to RCPassThru")
    set_servo_function(s1, ServoModes.RCPassThru)
    set_servo_function(s2, ServoModes.RCPassThru)
    set_servo_function(s3, ServoModes.RCPassThru)
    set_servo_function(s4, ServoModes.RCPassThru)

    print("Servo modes set")

    set_servo_pwm(s1, 1800)
    set_servo_pwm(s2, 1800)
    set_servo_pwm(s3, 1800)
    set_servo_pwm(s4, 1800)

    print("Servo PWM set")