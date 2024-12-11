import argparse
import asyncio

import pandas as pd
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError, VelocityNedYaw
from mavsdk.telemetry import LandedState


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--addr')
    parser.add_argument('-t', '--traj')
    parser.add_argument('-i', '--itvl', default=0.1, type=float)
    args = parser.parse_args()
    return args


def load_traj(path):
    traj = pd.read_csv(path).to_numpy()
    return traj


async def main():
    args = parse_args()

    traj = load_traj(args.traj)

    drone = System()

    print('Connecting to the drone...')
    await drone.connect(args.addr)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print('-- Connected to the drone!')
            break

    print('Checking global position and home position...')
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print('-- Global position and home position ok!')
            break

    print('-- Arming!')
    await drone.action.arm()

    print('Taking off...')
    await drone.action.takeoff()
    async for state in drone.telemetry.landed_state():
        if state is LandedState.IN_AIR:
            print('-- Drone has taken off!')
            break

    print('Starting offboard control...')
    await drone.offboard.set_position_ned(PositionNedYaw(*traj[0, 1:5]))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f'{error}')
        await drone.action.disarm()
        return

    print('Performing trajectory...')
    total = len(traj)
    i = 0
    t = 0
    while i < total:
        position = PositionNedYaw(*traj[i, 1:5])
        velocity = VelocityNedYaw(*traj[i, 5:9])
        await drone.offboard.set_position_velocity_ned(position, velocity)

        await asyncio.sleep(args.itvl)
        t += args.itvl

        while i < total and t > traj[i, 0]:
            i += 1
    print('-- Finished!')

    print('Landing...')
    await drone.action.return_to_launch()
    async for state in drone.telemetry.landed_state():
        if state is LandedState.ON_GROUND:
            print('-- Landed!')
            break

    print('Stopping offboard control...')
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f'{error}')
        await drone.action.disarm()
        return

    print('-- Disarming!')
    await drone.action.disarm()


if __name__ == '__main__':
    asyncio.run(main())
