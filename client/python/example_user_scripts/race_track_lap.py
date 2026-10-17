"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Project AirSim race track example.

Loads scene_race_track.jsonc (Unreal Chaos SUV + SimpleDrive) on the cartoon
oval level /Game/RaceTrack/RaceTrackMap and drives laps along the track
centerline with MoveOnPath. The centerline helpers are meant as a starting
point for AI racing (progress / cross-track-error rewards, reset poses).

Usage: open RaceTrackMap in the editor, press Play, then run
    python race_track_lap.py [--laps N] [--speed M/S]
"""

import argparse
import asyncio
import math
import time

from projectairsim import ProjectAirSimClient, Rover, World
from projectairsim.utils import projectairsim_log

# Stadium-shaped centerline in NED meters (measured from the Unreal level).
TRACK_CENTER_Y = -160.65
TURN_RADIUS = 132.75
STRAIGHT_HALF_LENGTH = 150.0
TRACK_WIDTH = 15.0
LAP_LENGTH = 4 * STRAIGHT_HALF_LENGTH + 2 * math.pi * TURN_RADIUS


def centerline_point(s: float, offset: float = 0.0) -> tuple:
    """NED (x, y) at arc length s from the start/finish (x=0 on the main
    straight), driving +x on the main straight. offset > 0 is toward the
    outside of the oval."""
    s %= LAP_LENGTH
    half_turn = math.pi * TURN_RADIUS
    r = TURN_RADIUS + offset
    if s < STRAIGHT_HALF_LENGTH:
        return (s, TRACK_CENTER_Y + r)
    s -= STRAIGHT_HALF_LENGTH
    if s < half_turn:
        a = s / TURN_RADIUS
        return (STRAIGHT_HALF_LENGTH + r * math.sin(a), TRACK_CENTER_Y + r * math.cos(a))
    s -= half_turn
    if s < 2 * STRAIGHT_HALF_LENGTH:
        return (STRAIGHT_HALF_LENGTH - s, TRACK_CENTER_Y - r)
    s -= 2 * STRAIGHT_HALF_LENGTH
    if s < half_turn:
        a = s / TURN_RADIUS
        return (-STRAIGHT_HALF_LENGTH - r * math.sin(a), TRACK_CENTER_Y - r * math.cos(a))
    s -= half_turn
    return (-STRAIGHT_HALF_LENGTH + s, TRACK_CENTER_Y + r)


def track_progress(x: float, y: float) -> tuple:
    """Project a NED position onto the centerline.

    Returns (arc length s in [0, LAP_LENGTH), signed cross-track error with
    positive toward the outside of the oval)."""
    half_turn = math.pi * TURN_RADIUS
    if abs(x) <= STRAIGHT_HALF_LENGTH:
        if y >= TRACK_CENTER_Y:  # main straight
            return ((x % LAP_LENGTH), y - (TRACK_CENTER_Y + TURN_RADIUS))
        return (
            STRAIGHT_HALF_LENGTH + half_turn + (STRAIGHT_HALF_LENGTH - x),
            (TRACK_CENTER_Y - TURN_RADIUS) - y,
        )
    cx = STRAIGHT_HALF_LENGTH if x > 0 else -STRAIGHT_HALF_LENGTH
    dx, dy = x - cx, y - TRACK_CENTER_Y
    dist = math.hypot(dx, dy)
    if x > 0:
        s = STRAIGHT_HALF_LENGTH + math.atan2(dx, dy) * TURN_RADIUS
    else:
        s = 3 * STRAIGHT_HALF_LENGTH + half_turn + math.atan2(-dx, -dy) * TURN_RADIUS
    return (s % LAP_LENGTH, dist - TURN_RADIUS)


def lap_waypoints(start_s: float, spacing: float = 10.0) -> list:
    n = int(LAP_LENGTH // spacing)
    return [list(centerline_point(start_s + spacing * (i + 1))) for i in range(n)]


def get_position(rover: Rover) -> tuple:
    kin = rover.get_ground_truth_kinematics()
    pos = kin.get("pose", {}).get("position", {}) if isinstance(kin, dict) else {}
    return (pos.get("x", 0.0), pos.get("y", 0.0))


async def main(laps: int, speed: float):
    client = ProjectAirSimClient()

    try:
        client.connect()
        world = World(client, "scene_race_track.jsonc", delay_after_load_sec=2)
        rover = Rover(client, world, "RaceCar")

        assert rover.enable_api_control()
        assert rover.arm()

        for lap in range(1, laps + 1):
            x, y = get_position(rover)
            s, cte = track_progress(x, y)
            projectairsim_log().info(
                f"Lap {lap}: start s={s:.1f} m, cross-track={cte:.2f} m"
            )
            t0 = time.time()
            # The timeout covers the whole path, so scale it with the lap time.
            move_task = await rover.move_on_path_async(
                path=lap_waypoints(s),
                velocity=speed,
                timeout_sec=2 * LAP_LENGTH / speed + 30.0,
            )
            await move_task
            projectairsim_log().info(f"Lap {lap} done in {time.time() - t0:.1f} s")

        brake_task = await rover.set_rover_controls(
            engine=0.0, steering_angle=0.0, brake=1.0
        )
        await brake_task
        await rover.wait_until_stopped_async(timeout_sec=10.0)

        rover.disarm()
        rover.disable_api_control()

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        client.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--speed", type=float, default=12.0)
    args = parser.parse_args()
    asyncio.run(main(args.laps, args.speed))
