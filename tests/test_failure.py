import asyncio
import random
from mavsdk import System, failure

# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
manual_inputs = [
    [0, 0, 0.5, 0],  # no movement
    [-1, 0, 0.5, 0],  # minimum roll
    [1, 0, 0.5, 0],  # maximum roll
    [0, -1, 0.5, 0],  # minimum pitch
    [0, 1, 0.5, 0],  # maximum pitch
    [0, 0, 0.5, -1],  # minimum yaw
    [0, 0, 0.5, 1],  # maximum yaw
    [0, 0, 1, 0],  # max throttle
    [0, 0, 0, 0],  # minimum throttle
]


async def manual_controls():
    print("mc 1")
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()
    # await drone.connect(system_address="udp://:14551")
    await drone.connect(system_address="udp://:14550")

    print("mc 2")
    # This waits till a mavlink based drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # await drone.failure.inject(failure.FailureUnit.SYSTEM_MOTOR, failure.FailureType.OK, 6)
    await print("mc 3")

if __name__ == "__main__":
    print("async start")
    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())