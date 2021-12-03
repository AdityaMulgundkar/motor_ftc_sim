import time
from pymavlink import mavutil
import enum
from scipy.interpolate import interp1d
import numpy as np

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
        print("PWM set to 0 for: ", servo.id)
    master.set_servo(channel=servo.id, pwm=pwm)

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

    pwm_1 = 1800
    pwm_3 = 1500
    pwm_4 = 1400

    print("Servo PWM set")

    ## Mappings for thrust to force
    ## Used a sample thruster/motor for force calculation:
    ## https://uav-en.tmotor.com/html/2018/u_0222/3.html
    ## Prop size: 12*4CF
    # 50% - 350 G
    # 65% - 550 G
    # 75% - 700 G
    # 85% - 870 G
    # 100% - 1000 G
    x =  [1000, 1500, 1650, 1750, 1850, 2000]
    y = [0, 350, 550, 700, 870, 1000]

    # Function that maps motor PWM to Force in grams
    pwm_to_force = interp1d(x, y, 'cubic')

    ## Since motor uses 12inch prop, min. required arm length = root(2) * half (arm length) = 1.414 * 100 cm - 30 > 0
    ## arm length = 100 cm gives ~10cm + space
    # Arm Length (l)
    l = arm_length = 1
    # Drag to Thrust co-efficients of the blade (12*4CF)
    d = drag_thrust_co_efficient = 4

    left_matrix = np.array([[1, 1, 1], [-l, l, 0], [d, d, -d]])
    right_matrix = np.array([pwm_to_force(pwm_1), pwm_to_force(pwm_3), pwm_to_force(pwm_4)])
    force_matrix = np.dot(left_matrix,right_matrix)

    print(force_matrix)